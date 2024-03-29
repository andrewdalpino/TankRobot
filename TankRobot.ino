#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <VL53L1X.h>
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#include <Ticker.h>
#include <Wire.h>
#include <FS.h>

#define SERIAL_BAUD 9600

#define BEEPER_PIN D7
#define BEEP_DURATION 100
#define BEEP_DELAY 30

#define AP_SSID "TankRobot"
#define AP_PASSWORD "KeepSummerSafe"
#define AP_CHANNEL 1
#define AP_MAX_CONNECTIONS 2
#define AP_HIDDEN 0

#define HTTP_PORT 80

#define HTTP_OK 200
#define HTTP_NOT_FOUND 404
#define HTTP_UNPROCESSABLE_ENTITY 422

#define BATTERY_PIN A0
#define BATTERY_SAMPLE_RATE 1000
#define VOLTMETER_RESOLUTION 1024.0
#define MIN_VOLTAGE 6.0
#define MAX_VOLTAGE 8.4

#define MOTOR_A_DIRECTION_PIN 0
#define MOTOR_A_THROTTLE_PIN 5
#define MOTOR_B_DIRECTION_PIN 2
#define MOTOR_B_THROTTLE_PIN 4
#define MIN_THROTTLE 800
#define MAX_THROTTLE 1000

#define FORWARD 1
#define REVERSE 0
#define LEFT 2
#define RIGHT 3

#define I2C_SDA_PIN D5
#define I2C_SCL_PIN D6
#define I2C_CLOCK 400000

#define MPU_ADDRESS 0x68
#define MPU_INTERRUPT_PIN 10
#define MPU_MAX_FIFO_COUNT 1024
#define LOW_PASS_FILTER_MODE 3
#define GYRO_CALIBRATION_LOOPS 8
#define QUATERNION_SAMPLE_RATE 20
#define GRAVITY_SAMPLE_RATE QUATERNION_SAMPLE_RATE
#define ORIENTATION_SAMPLE_RATE GRAVITY_SAMPLE_RATE
#define ACCEL_CALIBRATION_LOOPS 8
#define ACCEL_LSB_PER_G 16384.0
#define ACCELERATION_SAMPLE_RATE 50
#define IS_MOVING_SAMPLE_RATE ACCELERATION_SAMPLE_RATE
#define ZERO_MOVEMENT_THRESHOLD 0.13
#define ZERO_MOVEMENT_WINDOW 2
#define TEMPERATURE_SAMPLE_RATE 3000
#define TEMPERATURE_SENSITIVITY 340.0
#define TEMPERATURE_CONSTANT 36.53

#define ROTATOR_SAMPLE_RATE ORIENTATION_SAMPLE_RATE
#define ROTATOR_THRESHOLD 1.5 * DEG_TO_RAD

#define STABILIZER_SAMPLE_RATE ORIENTATION_SAMPLE_RATE

#define LIDAR_ADDRESS 0x52
#define LIDAR_TIMEOUT 500
#define LIDAR_SENSOR_OFFSET -10
#define LIDAR_TIMING_BUDGET 100000
#define LIDAR_SAMPLE_RATE 100
#define LIDAR_MAX_RANGE 3600
#define LIDAR_RANGE_VALID 0
#define LIDAR_NOISY_SIGNAL 1
#define LIDAR_SIGNAL_FAILURE 2
#define LIDAR_PHASE_OUT_OF_BOUNDS 4
#define VISIBILITY_SAMPLE_RATE 1000

#define EXPLORE_SAMPLE_RATE 250

#define SCAN_SAMPLE_RATE LIDAR_SAMPLE_RATE
#define SCAN_WINDOW 0.75 * PI
#define NUM_SCANS 5

#define MOVER_SAMPLE_RATE LIDAR_SAMPLE_RATE
#define MOVER_MAX_ACTIVATION 5000.0

#define COLLISION_SAMPLE_RATE LIDAR_SAMPLE_RATE
#define COLLISION_THRESHOLD 350

#define ROLLOVER_SAMPLE_RATE 500
#define ROLLOVER_THRESHOLD HALF_PI

#define MAX_RAND_INT 2147483647

Ticker beep_timer;

const IPAddress _ip(192, 168, 4, 1);
const IPAddress _gateway(192, 168, 4, 1);
const IPAddress _subnet(255, 255, 255, 0);

AsyncWebServer server(HTTP_PORT);

AsyncEventSource sensor_emitter("/events/robot/sensors");
AsyncEventSource training_emitter("/events/robot/training");
AsyncEventSource malfunction_emitter("/events/robot/malfunctions");

float _battery_voltage;

Ticker battery_timer;

uint8_t _direction = FORWARD;
unsigned int _throttle = 900;
bool _stopped = true;

Ticker brake_timer;

MPU6050 mpu(MPU_ADDRESS);

Quaternion _q;
VectorFloat _gravity;
VectorFloat _acceleration;
float _heading, _pitch, _roll;
float _heading_lock;
float _temperature;
bool _is_moving;
uint8_t _zero_movement_count;
uint8_t _dmp_fifo_buffer[64];
uint16_t _dmp_packet_size;
volatile bool _dmp_ready = false;
void IRAM_ATTR dmpDataReady();

Ticker quaternion_timer, gravity_timer, orientation_timer;
Ticker acceleration_timer, is_moving_timer;
Ticker temperature_timer;

float _rotator_p_gain = 2.0;
float _rotator_i_gain = 0.7;
float _rotator_d_gain = 1.4;

float _stabilizer_p_gain = 0.4;
float _stabilizer_i_gain = 0.8;
float _stabilizer_d_gain = 0.3;

float _rotator_prev_delta, _rotator_delta_integral;
float _stabilizer_prev_delta, _stabilizer_delta_integral;

Ticker rotator_timer, stabilizer_timer;
Ticker rollover_timer;

VL53L1X lidar;

float _scan_angles[NUM_SCANS];
float _angle_visibilities[NUM_SCANS];
unsigned int _distances_to_object[NUM_SCANS];
uint8_t _scan_direction = LEFT;
uint8_t _scan_count;

Ticker scan_timer, collision_timer, visibility_timer;

float _path_affinity = 2.0;

float _mover_learning_rate = 0.1;
float _mover_momentum = 0.9;
float _mover_alpha = 1e-4;
float _mover_max_overshoot = 0.2;
float _mover_features[4];
float _mover_weights[4];
float _mover_weight_velocities[4];
float _mover_bias;
float _mover_bias_velocity;
float _mover_prediction;
unsigned long _mover_start_timestamp;
unsigned long _mover_end_timestamp;
unsigned int _mover_epoch = 1;

Ticker mover_timer;

uint8_t _explorer_step;

Ticker explore_timer;

void forward(unsigned int duration = 0);
void reverse(unsigned int duration = 0);

/**
 * Set up the serial interface.
 */
void setupSerial()
{
    Serial.begin(SERIAL_BAUD);

    Serial.println("Serial port enabled");
}

/**
 * Setup the beeper.
 */
void setupBeeper()
{
    pinMode(BEEPER_PIN, OUTPUT);

    Serial.println("Beeper enabled");
}

/**
 * Set up the central processing unit.
 */
void setupCPU()
{
    Serial.print("CPU speed: ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println("mhz");
}

/**
 * Setup the flash ROM.
 */
void setupROM()
{
    if (!ESP.checkFlashCRC()) {
        Serial.println("Flash ROM CRC mismatch");

        panicNow();
    }

    Serial.print("ROM speed: ");
    Serial.print(ESP.getFlashChipSpeed() / 1000000);
    Serial.println("mhz");

    Serial.print("ROM size: ");
    Serial.print(ESP.getFlashChipRealSize() / 1024);
    Serial.println("kb");

    Serial.print("Free space: ");
    Serial.print(ESP.getFreeSketchSpace() / 1024.0, 1);
    Serial.println("kb");
}

/**
 * Set up the SPI Flash Filesystem.
 */
void setupSPIFFS()
{
    if (SPIFFS.begin()) {
        Serial.println("SPIFFS mounted");
    } else {
        Serial.println("Failed to mount SPIFFS");

        panicNow();
    }
}

/**
 * Set up the wireless access point.
 */
void setupAccessPoint()
{
    WiFi.softAPConfig(_ip, _gateway, _subnet);
    WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, AP_HIDDEN, AP_MAX_CONNECTIONS);

    Serial.println("Access point ready");
    Serial.print("SSID: ");
    Serial.println(AP_SSID);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("MAC Address: ");
    Serial.println(WiFi.softAPmacAddress());
}

/**
 * Set up the HTTP server.
 */
void setupHttpServer()
{
    setupStaticRoutes();

    setupApiRoutes();

    setupEventEmitters();

    server.onNotFound(handleNotFound);

    server.begin();

    Serial.print("HTTP server listening on port ");
    Serial.println(HTTP_PORT);
}

/**
 * Set up the HTTP static routes.
 */
void setupStaticRoutes()
{
    server.serveStatic("/ui", SPIFFS, "/app.html");
    server.serveStatic("/ui/control", SPIFFS, "/app.html");
    server.serveStatic("/ui/dynamics", SPIFFS, "/app.html");
    server.serveStatic("/ui/autonomy", SPIFFS, "/app.html");
    server.serveStatic("/ui/training", SPIFFS, "/app.html");
    server.serveStatic("/manifest.json", SPIFFS, "/manifest.json");
    server.serveStatic("/app.js", SPIFFS, "/app.js");
    server.serveStatic("/sw.js", SPIFFS, "/sw.js");
    server.serveStatic("/app.css", SPIFFS, "/app.css");
    server.serveStatic("/images/app-icon-small.png", SPIFFS, "/images/app-icon-small.png");
    server.serveStatic("/images/app-icon-large.png", SPIFFS, "/images/app-icon-large.png");
    server.serveStatic("/fonts/Roboto-300.woff2", SPIFFS, "/fonts/Roboto-300.woff2");
    server.serveStatic("/fonts/Roboto-regular.woff2", SPIFFS, "/fonts/Roboto-regular.woff2");
    server.serveStatic("/fonts/Roboto-500.woff2", SPIFFS, "/fonts/Roboto-500.woff2");
    server.serveStatic("/fonts/fa-solid-900.woff2", SPIFFS, "/fonts/fa-solid-900.woff2");
    server.serveStatic("/sounds/plucky.ogg", SPIFFS, "/sounds/plucky.ogg");
}

/**
 * Set up the HTTP API routes.
 */
void setupApiRoutes()
{
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/motors/direction", handleChangeDirection));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/motors/throttle", handleSetThrottlePercentage));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/rotator/left", handleRotateLeft));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/rotator/right", handleRotateRight));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/rotator/p", handleSetRotatorPGain));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/rotator/i", handleSetRotatorIGain));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/rotator/d", handleSetRotatorDGain));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/autonomy/path-affinity", handleSetPathAffinity));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/autonomy/mover/max-overshoot", handleSetMoverMaxOvershoot));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/autonomy/mover/learning-rate", handleSetMoverLearningRate));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/autonomy/mover/momentum", handleSetMoverMomentum));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/autonomy/mover/alpha", handleSetMoverAlpha));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/stabilizer/p", handleSetStabilizerPGain));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/stabilizer/i", handleSetStabilizerIGain));
    server.addHandler(new AsyncCallbackJsonWebHandler("/robot/stabilizer/d", handleSetStabilizerDGain));

    server.on("/robot", HTTP_GET, handleGetRobot);
    server.on("/robot/motors", HTTP_DELETE, handleStop);
    server.on("/robot/features/beeper", HTTP_PUT, handleBeep);
    server.on("/robot/autonomy/enabled", HTTP_PUT, handleExplore);
    server.on("/robot/autonomy", HTTP_DELETE, handleStop);
}

/**
 * Set up the HTTP event emitters.
 */
void setupEventEmitters()
{
    server.addHandler(&sensor_emitter);
    server.addHandler(&training_emitter);
    server.addHandler(&malfunction_emitter);
}

/**
 * Set up the battery voltmeter.
 */
void setupBattery()
{
    pinMode(BATTERY_PIN, INPUT);

    battery_timer.attach_ms(BATTERY_SAMPLE_RATE, updateBattery);

    Serial.println("Battery voltmeter enabled");
}

/**
 * Set up the motors.
 */
void setupMotors()
{
    pinMode(MOTOR_A_DIRECTION_PIN, OUTPUT);
    pinMode(MOTOR_A_THROTTLE_PIN, OUTPUT);

    pinMode(MOTOR_B_DIRECTION_PIN, OUTPUT);
    pinMode(MOTOR_B_THROTTLE_PIN, OUTPUT);

    analogWriteFreq(MAX_THROTTLE);

    Serial.println("Motors enabled");
}

/**
 * Set up the I2C bus.
 */
void setupI2C()
{
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    Wire.setClock(I2C_CLOCK);

    Serial.println("I2C bus initialized");
}

/**
 * Set up and calibrate the MPU accelerometer, gyroscope, and temperature sensor.
 */
void setupMPU()
{
    mpu.initialize();

    if (mpu.testConnection()) {
        Serial.println("MPU initialized");
    } else {
        Serial.println("MPU failure");

        panicNow();
    }

    pinMode(MPU_INTERRUPT_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);

    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    mpu.setDLPFMode(LOW_PASS_FILTER_MODE);

    Serial.print("Calibrating gyroscope ");
    mpu.CalibrateGyro(GYRO_CALIBRATION_LOOPS);
    Serial.println(" done");

    Serial.print("Calibrating accelerometer ");
    mpu.CalibrateAccel(ACCEL_CALIBRATION_LOOPS);
    Serial.println(" done");

    mpu.setSleepEnabled(false);

    mpu.dmpInitialize();

    mpu.setDMPEnabled(true);

    _dmp_packet_size = mpu.dmpGetFIFOPacketSize();

    quaternion_timer.attach_ms(QUATERNION_SAMPLE_RATE, updateQuaternion);
    gravity_timer.attach_ms(GRAVITY_SAMPLE_RATE, updateGravity);
    orientation_timer.attach_ms(ORIENTATION_SAMPLE_RATE, updateOrientation);
    acceleration_timer.attach_ms(ACCELERATION_SAMPLE_RATE, updateAcceleration);
    is_moving_timer.attach_ms(IS_MOVING_SAMPLE_RATE, updateIsMoving);
    temperature_timer.attach_ms(TEMPERATURE_SAMPLE_RATE, updateTemperature);

    Serial.println("DMP initialized");
}

/**
 * Set up the lidar distance sensor.
 */
void setupLidar()
{
    lidar.setTimeout(LIDAR_TIMEOUT);

    if (lidar.init()) {
        Serial.println("Lidar initialized");
    } else {
        Serial.println("Lidar failure");

        panicNow();
    }

    lidar.setDistanceMode(VL53L1X::Long);
    lidar.setMeasurementTimingBudget(LIDAR_TIMING_BUDGET);
    lidar.startContinuous(LIDAR_SAMPLE_RATE);

    visibility_timer.attach_ms(VISIBILITY_SAMPLE_RATE, updateVisibility);
}

/**
 * Set up the random number generator.
 */
void setupRandom()
{
    randomSeed(RANDOM_REG32);

    Serial.println("Seeded random number generator");
}

/**
 * Set up the mover model.
 */
void setupMover()
{
    for (uint8_t i = 0; i < 5; ++i) {
        _mover_weights[i] = random(-MAX_RAND_INT, MAX_RAND_INT) / (float)MAX_RAND_INT;
    }

    Serial.println("Mover model initialized");
}

/**
 * Bootstrap the robot.
 */
void setup()
{
    setupSerial();

    setupBeeper();

    setupCPU();
    setupROM();

    setupSPIFFS();

    setupAccessPoint();
    setupHttpServer();

    setupBattery();

    setupMotors();

    setupI2C();
    setupMPU();
    setupLidar();

    setupRandom();

    setupMover();

    Serial.println("Ready");
}

/**
 * The main event loop.
 */
void loop()
{
    if (lidar.dataReady()) {
        lidar.read(false);
    }

    yield();

    if (_dmp_ready) {
        uint8_t mpu_int_status = mpu.getIntStatus();

        uint16_t fifo_count = mpu.getFIFOCount();

        if (mpu_int_status & 0x10 || fifo_count >= MPU_MAX_FIFO_COUNT) {
            mpu.resetFIFO();
        } else if (fifo_count % _dmp_packet_size != 0) {
            mpu.resetFIFO();
        } else if (mpu_int_status & 0x02) {
            while (fifo_count >= _dmp_packet_size) {
                mpu.getFIFOBytes(_dmp_fifo_buffer, _dmp_packet_size);

                fifo_count -= _dmp_packet_size;
            }
        }

        _dmp_ready = false;
    }
}

/**
 * Handle a get robot info request.
 */
void handleGetRobot(AsyncWebServerRequest *request)
{
    StaticJsonDocument<768> doc;
    char buffer[768];

    JsonObject robot = doc.createNestedObject("robot");

    JsonObject motors = robot.createNestedObject("motors");

    motors["direction"] = _direction;
    motors["throttle"] = throttlePercentage();
    motors["stopped"] = _stopped;

    JsonObject sensors = robot.createNestedObject("sensors");

    JsonObject battery = sensors.createNestedObject("battery");

    battery["voltage"] = _battery_voltage;
    battery["capacity"] = batteryCapacity();

    JsonObject lidar = sensors.createNestedObject("lidar");

    lidar["visibility"] = visibility();

    sensors["temperature"] = _temperature;

    JsonObject rotator = robot.createNestedObject("rotator");

    rotator["p"] = _rotator_p_gain;
    rotator["i"] = _rotator_i_gain;
    rotator["d"] = _rotator_d_gain;

    JsonObject autonomy = robot.createNestedObject("autonomy");

    autonomy["enabled"] = isAutonomous();
    autonomy["pathAffinity"] = _path_affinity;

    JsonObject mover = autonomy.createNestedObject("mover");

    mover["learningRate"] = _mover_learning_rate;
    mover["momentum"] = _mover_momentum;
    mover["alpha"] = _mover_alpha;
    mover["maxOvershoot"] = _mover_max_overshoot;

    JsonObject importances = mover.createNestedObject("importances");

    importances["throttle"] = moverThrottleImportance();
    importances["battery"] = moverBatteryImportance();
    importances["pitch"] = moverPitchImportance();
    importances["distance"] = moverDistanceImportance();

    JsonObject stabilizer = robot.createNestedObject("stabilizer");

    stabilizer["p"] = _stabilizer_p_gain;
    stabilizer["i"] = _stabilizer_i_gain;
    stabilizer["d"] = _stabilizer_d_gain;

    serializeJson(doc, buffer);

    request->send(HTTP_OK, "application/json", buffer);
}

/**
 * Handle a change direction request.
 */
void handleChangeDirection(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("direction")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    uint8_t direction = doc["direction"];

    switch (direction) {
        case FORWARD:
            forward();

            break;

        case LEFT:
            left();

            break;

        case RIGHT:
            right();

            break;

        case REVERSE:
            reverse();

            break;
    }

    request->send(HTTP_OK);
}

/**
 * Handle the stop movement request.
 */
void handleStop(AsyncWebServerRequest *request)
{
    stop();

    request->send(HTTP_OK);
}

/**
 * Handle a beep request.
 */
void handleBeep(AsyncWebServerRequest *request)
{
    beep(2);

    request->send(HTTP_OK);
}

/**
 * Handle the explore request.
 */
void handleExplore(AsyncWebServerRequest *request)
{
    explore();

    request->send(HTTP_OK);
}

/**
 * Handle a set throttle position request.
 */
void handleSetThrottlePercentage(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("throttle")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    setThrottlePercentage(doc["throttle"]);

    request->send(HTTP_OK);
}

/**
 * Handle the rotate left request.
 */
void handleRotateLeft(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("radians")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    rotateLeft(doc["radians"]);

    request->send(HTTP_OK);
}

/**
 * Handle the rotate right request.
 */
void handleRotateRight(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("radians")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    rotateRight(doc["radians"]);

    request->send(HTTP_OK);
}

/**
 * Handle the set rotator proportional control gain.
 */
void handleSetRotatorPGain(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("gain")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    setRotatorPGain(doc["gain"]);

    request->send(HTTP_OK);
}

/**
 * Handle the set rotator integral control gain.
 */
void handleSetRotatorIGain(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("gain")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    setRotatorIGain(doc["gain"]);

    request->send(HTTP_OK);
}

/**
 * Handle the set rotator derivative control gain.
 */
void handleSetRotatorDGain(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("gain")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    setRotatorDGain(doc["gain"]);

    request->send(HTTP_OK);
}

/**
 * Handle the set path affinity request.
 */
void handleSetPathAffinity(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("pathAffinity")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    setPathAffinity(doc["pathAffinity"]);

    request->send(HTTP_OK);
}

/**
 * Handle the set mover max overshoot request.
 */
void handleSetMoverMaxOvershoot(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("maxOvershoot")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    setMoverMaxOvershoot(doc["maxOvershoot"]);

    request->send(HTTP_OK);
}

/**
 * Handle the set mover learning rate request.
 */
void handleSetMoverLearningRate(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("learningRate")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    setMoverLearningRate(doc["learningRate"]);

    request->send(HTTP_OK);
}

/**
 * Handle the set mover momentum request.
 */
void handleSetMoverMomentum(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("momentum")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    setMoverMomentum(doc["momentum"]);

    request->send(HTTP_OK);
}

/**
 * Handle the set mover L2 regularization parameter.
 */
void handleSetMoverAlpha(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("alpha")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    setMoverAlpha(doc["alpha"]);

    request->send(HTTP_OK);
}

/**
 * Handle the set stabilizer proportional control gain.
 */
void handleSetStabilizerPGain(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("gain")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    setStabilizerPGain(doc["gain"]);

    request->send(HTTP_OK);
}

/**
 * Handle the set stabilizer integral control gain.
 */
void handleSetStabilizerIGain(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("gain")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    setStabilizerIGain(doc["gain"]);

    request->send(HTTP_OK);
}

/**
 * Handle the set stabilizer derivative control gain.
 */
void handleSetStabilizerDGain(AsyncWebServerRequest *request, JsonVariant &json)
{
    const JsonObject &doc = json.as<JsonObject>();

    if (!doc.containsKey("gain")) {
        request->send(HTTP_UNPROCESSABLE_ENTITY);

        return;
    }

    setStabilizerDGain(doc["gain"]);

    request->send(HTTP_OK);
}

/**
 * Catch all route responds with 404.
 */
void handleNotFound(AsyncWebServerRequest *request)
{
    request->send(HTTP_NOT_FOUND);
}

/**
 * Move forward for an amount of milliseconds.
 */
void forward(unsigned int duration)
{
    _direction = FORWARD;

    enableCollisionPrevention();
    enableStabilizer();

    digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
    digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

    if (duration > 0) {
        brake_timer.once_ms(duration, brake);
    }

    go();
}

/**
 * Turn left.
 */
void left()
{
    _direction = LEFT;

    digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
    digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);

    go();
}

/**
 * Turn right.
 */
void right()
{
    _direction = RIGHT;

    digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
    digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

    go();
}

/**
 * Move in reverse for a given number of milliseconds.
 */
void reverse(unsigned int duration)
{
    _direction = REVERSE;

    enableStabilizer();

    digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
    digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);

    if (duration > 0) {
        brake_timer.once_ms(duration, brake);
    }

    go();
}

/**
 * Engage the motors.
 */
void go()
{
    _stopped = false;

    enableRolloverDetection();

    analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
    analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);
}

/**
 * Disengage the motors and related timers.
 */
void stop()
{
    brake();

    _stopped = true;

    explore_timer.detach();
    rotator_timer.detach();
    scan_timer.detach();
    mover_timer.detach();
}

/**
 * Apply the brake to the motors.
 */
void brake()
{
    disableCollisionPrevention();
    disableRolloverDetection();
    disableStabilizer();

    analogWrite(MOTOR_A_THROTTLE_PIN, 0);
    analogWrite(MOTOR_B_THROTTLE_PIN, 0);
}

/**
 * Returns the current power capacity of the battery.
 */
float batteryCapacity()
{
    return (_battery_voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE);
}

/**
 * Returns the throttle position as a percentage.
 */
float throttlePercentage()
{
    return (_throttle - MIN_THROTTLE) / (float)(MAX_THROTTLE - MIN_THROTTLE) * 100.0;
}

/**
 * Set the throttle percentage of the motor controller.
 */
void setThrottlePercentage(int throttle)
{
    _throttle = map(throttle, 0, 100, MIN_THROTTLE, MAX_THROTTLE);

    if (!_stopped) {
        analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
        analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);
    }
}

/**
 * Set the path affinity.
 */
void setPathAffinity(float pathAffinity)
{
    _path_affinity = constrain(pathAffinity, 1.0, 3.0);
}

/**
 * Set the mover max overshoot.
 */
void setMoverMaxOvershoot(float maxOvershoot)
{
    _mover_max_overshoot = constrain(maxOvershoot, 0.0, 1.0);
}

/**
 * Set the mover learning rate.
 */
void setMoverLearningRate(float learningRate)
{
    _mover_learning_rate = fmax(0.0, learningRate);
}

/**
 * Set mover momentum amount.
 */
void setMoverMomentum(float momentum)
{
    _mover_momentum = constrain(momentum, 0.0, 1.0);
}

/**
 * Set the mover L@ regularization term.
 */
void setMoverAlpha(float alpha)
{
    _mover_alpha = fmax(0.0, alpha);
}

/**
 * Rotate the vehicle to the left.
 */
void rotateLeft(float radians)
{
    _direction = LEFT;

    rotate(-radians);

    digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
    digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);
}

/**
 * Rotate the vehicle to the right.
 */
void rotateRight(float radians)
{
    _direction = RIGHT;

    rotate(radians);

    digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
    digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);
}

/**
 * Rotate the vehicle.
 */
void rotate(float radians)
{
    float target = constrain(radians, -PI, PI);

    _heading_lock = calculateTargetAngle(_heading, target);

    _rotator_delta_integral = 0.0;
    _rotator_prev_delta = 0.0;

    rotator_timer.attach_ms(ROTATOR_SAMPLE_RATE, rotator);
}

/**
 * Set the rotator proportional control gain.
 */
void setRotatorPGain(float gain)
{
    _rotator_p_gain = fmax(0.0, gain);
}

/**
 * Set the rotator integral control gain.
 */
void setRotatorIGain(float gain)
{
    _rotator_i_gain = fmax(0.0, gain);
}

/**
 * Set the rotator derivative control gain.
 */
void setRotatorDGain(float gain)
{
    _rotator_d_gain = fmax(0.0, gain);
}

/**
 * Control loop to rotate the vehicle by actuating the motors.
 */
void rotator()
{
    float delta = calculateAngleDelta(_heading, _heading_lock);

    _rotator_delta_integral += 0.5 * (delta + _rotator_prev_delta);

    _rotator_delta_integral = constrain(_rotator_delta_integral, -PI, PI);

    float derivative = delta - _rotator_prev_delta;

    float theta = _rotator_p_gain * delta + _rotator_i_gain * _rotator_delta_integral + _rotator_d_gain * derivative;

    float alpha = min(fabs(theta) / PI, 1.0);

    int rotator_throttle = round(alpha * _throttle);

    rotator_throttle = max(MIN_THROTTLE, rotator_throttle);

    if (theta < 0.0) {
        if (_direction != LEFT) {
            digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
            digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);

            _direction = LEFT;
        }
    } else {
        if (_direction != RIGHT) {
            digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
            digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

            _direction = RIGHT;
        }
    }

    analogWrite(MOTOR_A_THROTTLE_PIN, rotator_throttle);
    analogWrite(MOTOR_B_THROTTLE_PIN, rotator_throttle);

    if (fabs(delta) < ROTATOR_THRESHOLD) {
        brake();

        rotator_timer.detach();
    } else {
        _rotator_prev_delta = delta;
    }
}

/**
 * Kick off the explore loop.
 */
void explore()
{
    stop();

    _explorer_step = 0;

    explore_timer.attach_ms(EXPLORE_SAMPLE_RATE, explorer);
}

/**
 * Is the robot acting in autonomous mode?
 */
bool isAutonomous()
{
    return explore_timer.active();
}

/**
 * Control loop to explore the environment.
 */
void explorer()
{
    bool ready = !scan_timer.active() && !mover_timer.active() && !rotator_timer.active();

    if (ready && !_is_moving) {
        switch (_explorer_step) {
            case 0:
            {
                scanEnvironment();

                _explorer_step = 1;
            }

            break;

            case 1:
            {
                choosePath();

                _explorer_step = 2;
            }

            break;

            case 2:
            {
                move();

                _explorer_step = 0;
            }

            break;
        }
    }
}

/**
 * Scan the environment.
 */
void scanEnvironment()
{
    _scan_direction = random(LEFT, RIGHT);

    if (_scan_direction == LEFT) {
        rotateRight(0.5 * SCAN_WINDOW);
    } else {
        rotateLeft(0.5 * SCAN_WINDOW);
    }

    _scan_count = 0;

    scan_timer.attach_ms(SCAN_SAMPLE_RATE, scanner);
}

/**
 * Control loop to scan the environment for objects.
 */
void scanner()
{
    float radians;

    if (!rotator_timer.active() && !_is_moving) {
        _scan_angles[_scan_count] = _heading;
        _angle_visibilities[_scan_count] = visibility();
        _distances_to_object[_scan_count] = distanceToObject();

        ++_scan_count;

        if (_scan_count < NUM_SCANS) {
            radians = SCAN_WINDOW / (NUM_SCANS - 1);

            if (_scan_direction == LEFT) {
                rotateLeft(radians);
            } else {
                rotateRight(radians);
            }
        } else {
            scan_timer.detach();
        }
    }
}

/**
 * Choose a path to traverse.
 */
void choosePath()
{
    long weights[NUM_SCANS];

    float scale = 2.0 / (NUM_SCANS * _path_affinity);

    float score;

    for (uint8_t i = 0; i < NUM_SCANS; ++i) {
        score = scale * _angle_visibilities[i] * _distances_to_object[i];

        weights[i] = round(pow(score, _path_affinity));
    }

    long total = 0;

    for (uint8_t i = 0; i < NUM_SCANS; ++i) {
        total += weights[i];
    }

    long threshold = random(0, total);

    float delta;

    for (uint8_t i = 0; i < NUM_SCANS; ++i) {
        threshold -= weights[i];

        if (threshold <= 0) {
            delta = calculateAngleDelta(_heading, _scan_angles[i]);

            break;
        }
    }

    rotate(delta);
}

/**
 * Displace the vehicle along the x axis.
 */
void move()
{
    _mover_features[0] = throttlePercentage() / 100.0;
    _mover_features[1] = batteryCapacity() / 100.0;
    _mover_features[2] = _pitch / HALF_PI;
    _mover_features[3] = distanceToObject() / (float) LIDAR_MAX_RANGE;

    float z = dot(_mover_features, _mover_weights, 4);

    z += _mover_bias;

    float activation = fmax(0.0, fmin(z, MOVER_MAX_ACTIVATION));

    unsigned long now = millis();

    long max_overshoot = round(_mover_max_overshoot * fabs(z));

    long overshoot = random(0, max_overshoot);

    long burn_time = round(activation) + overshoot;

    _mover_start_timestamp = now;
    _mover_end_timestamp = now + burn_time;

    _mover_prediction = activation;

    _direction = FORWARD;

    mover_timer.attach_ms(MOVER_SAMPLE_RATE, mover);

    enableStabilizer();

    digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
    digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

    go();
}

/**
 * Control loop to displace the vehicle in the x axis.
 */
void mover()
{
    unsigned long now = millis();

    bool collision = distanceToObject() < COLLISION_THRESHOLD;

    if (collision || now > _mover_end_timestamp) {
        brake();

        mover_timer.detach();

        float delta = now - _mover_start_timestamp;

        float dydl = _mover_prediction - delta;

        float dydw;

        for (uint8_t i = 0; i < 4; ++i) {
            dydw = dydl * _mover_features[i];

            dydw += _mover_alpha * _mover_weights[i];

            _mover_weight_velocities[i] = _mover_learning_rate * dydw + _mover_momentum * _mover_weight_velocities[i];

            _mover_weights[i] -= _mover_learning_rate * dydw + _mover_momentum * _mover_weight_velocities[i];
        }

        float dydb = dydl;

        _mover_bias_velocity = _mover_learning_rate * dydb + _mover_momentum * _mover_bias_velocity;

        _mover_bias -= _mover_learning_rate * dydb + _mover_momentum * _mover_bias_velocity;

        ++_mover_epoch;

        if (training_emitter.count() > 0) {
            StaticJsonDocument<256> doc;
            char buffer[256];

            float loss = sq(dydl);

            doc["epoch"] = _mover_epoch;
            doc["loss"] = loss;

            JsonObject importances = doc.createNestedObject("importances");

            importances["throttle"] = moverThrottleImportance();
            importances["battery"] = moverBatteryImportance();
            importances["pitch"] = moverPitchImportance();
            importances["distance"] = moverDistanceImportance();

            serializeJson(doc, buffer);

            training_emitter.send(buffer, "mover-epoch-complete");
        }
    }

    if (collision) {
        reverse(300);
    }
}

/**
 * Return the importance of the throttle feature in the mover model.
 */
float moverThrottleImportance()
{
    return fabs(_mover_weights[0]);
}

/**
 * Return the importance of the battery feature in the mover model.
 */
float moverBatteryImportance()
{
    return fabs(_mover_weights[1]);
}

/**
 * Return the importance of the pitch feature in the mover model.
 */
float moverPitchImportance()
{
    return fabs(_mover_weights[2]);
}

/**
 * Return the importance of the distance feature in the mover model.
 */
float moverDistanceImportance()
{
    return fabs(_mover_weights[3]);
}

/**
 * Enable linear movement stabilizer.
 */
void enableStabilizer()
{
    _heading_lock = _heading;

    _stabilizer_delta_integral = 0.0;
    _stabilizer_prev_delta = 0.0;

    stabilizer_timer.attach_ms(STABILIZER_SAMPLE_RATE, stabilize);
}

/**
 * Set the stabilizer proportional control gain.
 */
void setStabilizerPGain(float gain)
{
    _stabilizer_p_gain = fmax(0.0, gain);
}

/**
 * Set the stabilizer integral control gain.
 */
void setStabilizerIGain(float gain)
{
    _stabilizer_i_gain = fmax(0.0, gain);
}

/**
 * Set the stabilizer derivative control gain.
 */
void setStabilizerDGain(float gain)
{
    _stabilizer_d_gain = fmax(0.0, gain);
}

/**
 * Disable the linear stabilizer.
 */
void disableStabilizer()
{
    stabilizer_timer.detach();
}

/**
 * Stabilize linear movement using orientation feedback from the gyroscope.
 */
void stabilize()
{
    float delta = calculateAngleDelta(_heading, _heading_lock);

    _stabilizer_delta_integral += 0.5 * (delta + _stabilizer_prev_delta);

    _stabilizer_delta_integral = constrain(_stabilizer_delta_integral, -PI, PI);

    float derivative = delta - _stabilizer_prev_delta;

    float theta = _stabilizer_p_gain * delta + _stabilizer_i_gain * _stabilizer_delta_integral + _stabilizer_d_gain * derivative;

    float beta = min(fabs(theta) / PI, 1.0);

    int backoff_throttle = round((1.0 - beta) * _throttle);

    backoff_throttle = max(MIN_THROTTLE, backoff_throttle);

    if (theta < 0.0) {
        switch (_direction) {
            case FORWARD:
                analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
                analogWrite(MOTOR_B_THROTTLE_PIN, backoff_throttle);

                break;

            case REVERSE:
                analogWrite(MOTOR_A_THROTTLE_PIN, backoff_throttle);
                analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);

                break;
        }
    } else {
        switch (_direction) {
            case FORWARD:
                analogWrite(MOTOR_A_THROTTLE_PIN, backoff_throttle);
                analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);

                break;

            case REVERSE:
                analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
                analogWrite(MOTOR_B_THROTTLE_PIN, backoff_throttle);

                break;
        }
    }

    _stabilizer_prev_delta = delta;
}

/**
 * Enable rollover detection.
 */
void enableRolloverDetection()
{
    rollover_timer.attach_ms(ROLLOVER_SAMPLE_RATE, detectRollover);
}

/**
 * Disable rollover detection.
 */
void disableRolloverDetection()
{
    rollover_timer.detach();
}

/**
 * Detect if the vehicle has rolled over.
 */
void detectRollover()
{
    bool rollover = fabs(_pitch) > ROLLOVER_THRESHOLD || fabs(_roll) > ROLLOVER_THRESHOLD;

    if (rollover) {
        stop();

        beep(5);

        malfunction_emitter.send("", "rollover-detected");
    }
}

/**
 * Enable collision detection.
 */
void enableCollisionPrevention()
{
    collision_timer.attach_ms(COLLISION_SAMPLE_RATE, detectCollision);
}

/**
 * Disable collision detection.
 */
void disableCollisionPrevention()
{
    collision_timer.detach();
}

/**
 * Detect if collision with an object is iminent and stop the vehicle.
 */
void detectCollision()
{
    if (distanceToObject() < COLLISION_THRESHOLD) {
        brake();

        malfunction_emitter.send("", "collision-detected");
    }
}

/**
 * Update the battery voltage.
 */
void updateBattery()
{
    unsigned int raw = analogRead(BATTERY_PIN);

    _battery_voltage = (raw / VOLTMETER_RESOLUTION) * MAX_VOLTAGE;

    if (_battery_voltage > 1.0 && _battery_voltage < MIN_VOLTAGE) {
        stop();

        if (malfunction_emitter.count() > 0) {
            StaticJsonDocument<64> doc;
            char buffer[64];

            doc["voltage"] = _battery_voltage;

            serializeJson(doc, buffer);

            malfunction_emitter.send(buffer, "battery-undervoltage");
        }

        panicNow();
    }
    else
    {
        if (sensor_emitter.count() > 0) {
            StaticJsonDocument<64> doc;
            char buffer[64];

            doc["voltage"] = _battery_voltage;
            doc["capacity"] = batteryCapacity();

            serializeJson(doc, buffer);

            sensor_emitter.send(buffer, "battery-voltage-updated");
        }
    }
}

/**
 * Update the temperature.
 */
void updateTemperature()
{
    _temperature = mpu.getTemperature() / TEMPERATURE_SENSITIVITY + TEMPERATURE_CONSTANT;

    if (sensor_emitter.count() > 0) {
        StaticJsonDocument<64> doc;
        char buffer[64];

        doc["temperature"] = _temperature;

        serializeJson(doc, buffer);

        sensor_emitter.send(buffer, "temperature-updated");
    }
}

/**
 * Update the lidar visibility reading.
 */
void updateVisibility()
{
    if (sensor_emitter.count() > 0) {
        StaticJsonDocument<64> doc;
        char buffer[64];

        doc["visibility"] = visibility();

        serializeJson(doc, buffer);

        sensor_emitter.send(buffer, "visibility-updated");
    }
}

/**
 * Beep the beeper.
 */
void beep(uint8_t count)
{
    digitalWrite(BEEPER_PIN, HIGH);

    --count;

    beep_timer.once_ms(BEEP_DURATION, silenceBeeper, count);
}

/**
 * Silence the beeper.
 */
void silenceBeeper(uint8_t remaining)
{
    digitalWrite(BEEPER_PIN, LOW);

    if (remaining > 0) {
        beep_timer.once_ms(BEEP_DELAY, beep, remaining);
    }
}

/**
 * Update the rotation of the vehicle.
 */
void updateQuaternion()
{
    mpu.dmpGetQuaternion(&_q, _dmp_fifo_buffer);
}

/**
 * Update the gravity vector from the raw quaternion.
 */
void updateGravity()
{
    _gravity.x = -2.0 * (_q.x * _q.z - _q.w * _q.y);
    _gravity.y = 2.0 * (_q.w * _q.x + _q.y * _q.z);
    _gravity.z = -(sq(_q.w) - sq(_q.x) - sq(_q.y) + sq(_q.z));
}

/**
 * Calculate the current heading, pitch, and roll from the raw quaternion and gravity vector.
 */
void updateOrientation()
{
    _heading = atan2(2.0 * -_q.x * _q.y - 2.0 * _q.w * -_q.z, 2.0 * sq(_q.w) + 2.0 * sq(_q.x) - 1.0);

    _pitch = atan2(_gravity.x, sqrt(sq(_gravity.y) + sq(_gravity.z)));

    _roll = atan2(_gravity.y, _gravity.z);
}

/**
 * Update the current acceleration in m/sec ^ 2 of the vehicle with gravity vector removed.
 */
void updateAcceleration()
{
    VectorInt16 aa;

    mpu.dmpGetAccel(&aa, _dmp_fifo_buffer);

    _acceleration.x = -(aa.x / ACCEL_LSB_PER_G);
    _acceleration.y = (aa.y / ACCEL_LSB_PER_G);
    _acceleration.z = -(aa.z / ACCEL_LSB_PER_G);

    _acceleration.x -= _gravity.x;
    _acceleration.y -= _gravity.y;
    _acceleration.z -= _gravity.z;
}

/**
 * Is the vehicle in motion?
 */
bool updateIsMoving()
{
    float norm = sqrt(sq(_acceleration.x) + sq(_acceleration.y) + sq(_acceleration.z));

    if (norm > ZERO_MOVEMENT_THRESHOLD) {
        _zero_movement_count = 0;

        _is_moving = true;
    } else {
        if (_zero_movement_count >= ZERO_MOVEMENT_WINDOW) {
            _is_moving = false;
        } else {
            ++_zero_movement_count;

            _is_moving = true;
        }
    }
}

/**
 * Return the distance to object using the narrow beam.
 */
unsigned int distanceToObject()
{
    int distance;

    switch (lidar.ranging_data.range_status) {
        case LIDAR_RANGE_VALID:
        case LIDAR_NOISY_SIGNAL:
            distance = lidar.ranging_data.range_mm;

            break;

        case LIDAR_SIGNAL_FAILURE:
        case LIDAR_PHASE_OUT_OF_BOUNDS:
            distance = LIDAR_MAX_RANGE;

            break;
    }

    distance += LIDAR_SENSOR_OFFSET;

    return max(0, distance);
}

/**
 * Return the current lidar visibility.
 */
float visibility()
{
    float signal = lidar.ranging_data.peak_signal_count_rate_MCPS;

    float noise = lidar.ranging_data.ambient_count_rate_MCPS;

    float total = signal + noise;

    return signal / total;
}

/**
 * Calculate the final angle of a rotation from a starting point with the addition of delta in radians.
 */
float calculateTargetAngle(float start, float delta)
{
    float target = start + delta;

    if (target > PI) {
        target -= TWO_PI;
    } else if (target < -PI) {
        target += TWO_PI;
    }

    return target;
}

/**
 * Calculate the difference between a target and the current heading.
 */
float calculateAngleDelta(float start, float target)
{
    float delta = target - start;

    if (fabs(delta) > PI) {
        if (start > 0.0) {
            delta = target - (start - TWO_PI);
        } else {
            delta = target - (start + TWO_PI);
        }
    }

    return delta;
}

/**
 * Calculate the dot product of two equi-length arrays.
 */
float dot(float a[], float b[], uint8_t n)
{
    float sigma = 0.0;

    for (uint8_t i = 0; i < n; ++i) {
        sigma += a[i] * b[i];
    }

    return sigma;
}

/**
 * Return the median of a set of unsigned integers.
*/
unsigned int median(unsigned int values[], uint8_t n)
{
    bubbleSort(values)

    uint8_t mid = n / 2

    if (n % 2 == 0) {
        median = 0.5 * (values[mid - 1] + values[mid]);
    } else {
        median = values[mid];
    }

    return median;
}

/**
 * Bubble sort an array of unsigned integers.
 */
void bubbleSort(unsigned int values[], uint8_t n)
{
    unsigned int temp;
    uint8_t k;
    bool swapped;
   
    for (uint8_t i = 0; i < n - 1; i++) {
        swapped = false;
     
        for (uint8_t j = 0; j < n - i - 1; j++) {
            k = j + 1;
      
            if (values[j] > values[k]) {
                temp = values[j];

                values[j] = values[k];
                values[k] = temp;
           
                swapped = true;
            }
        } 
   
        if (!swapped) {
            break; 
        }
    }
}

/**
 * What to do when we don't know what to do.
 */
void panicNow()
{
    Serial.println("WTF panicking!");

    beep(4);

    delay(4 * (BEEP_DURATION + BEEP_DELAY));

    ESP.deepSleep(0);
}

/**
 * MPU interrupt callback to signal DMP data is ready in the buffer.
 */
void dmpDataReady()
{
    _dmp_ready = true;
}

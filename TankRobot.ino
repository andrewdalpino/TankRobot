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
#define MIN_THROTTLE 700
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
#define GRAVITY_SAMPLE_RATE 10
#define ORIENTATION_SAMPLE_RATE 10
#define ACCEL_CALIBRATION_LOOPS 8
#define TEMPERATURE_SAMPLE_RATE 1000
#define TEMPERATURE_SENSITIVITY 340.0
#define TEMPERATURE_CONSTANT 36.53

#define ROTATOR_SAMPLE_RATE ORIENTATION_SAMPLE_RATE
#define ROTATOR_THRESHOLD 1.3 * DEG_TO_RAD
#define ROTATOR_P_GAIN 2.0
#define ROTATOR_I_GAIN 0.7
#define ROTATOR_D_GAIN 1.4

#define STABILIZER_SAMPLE_RATE ORIENTATION_SAMPLE_RATE
#define STABILIZER_P_GAIN 0.6
#define STABILIZER_I_GAIN 0.8
#define STABILIZER_D_GAIN 0.3

#define LIDAR_ADDRESS 0x52
#define LIDAR_TIMEOUT 500
#define LIDAR_SENSOR_OFFSET -10
#define LIDAR_TIMING_BUDGET 100000
#define LIDAR_SAMPLE_RATE 100
#define LIDAR_MAX_RANGE 4000
#define LIDAR_RANGE_VALID 0
#define LIDAR_NOISY_SIGNAL 1
#define LIDAR_SIGNAL_FAILURE 2
#define LIDAR_PHASE_OUT_OF_BOUNDS 4

#define EXPLORE_SAMPLE_RATE 500

#define SCAN_SAMPLE_RATE 100
#define SCAN_WINDOW M_PI
#define NUM_SCANS 4

#define PATH_AFFINITY 2.0

#define MOVER_SAMPLE_RATE LIDAR_SAMPLE_RATE
#define MOVER_MAX_OVERSHOOT 0.33
#define MOVER_LEARNING_RATE 10.0
#define MOVER_MOMENTUM_DECAY 0.1
#define MOVER_NORM_DECAY 0.001
#define MOVER_ALPHA 1e-4

#define COLLISION_SAMPLE_RATE LIDAR_SAMPLE_RATE
#define COLLISION_THRESHOLD 400

#define ROLLOVER_SAMPLE_RATE 500
#define ROLLOVER_THRESHOLD HALF_PI

#define MAX_RAND_INT 2147483647

Ticker beep_timer;

const IPAddress _ip(192,168,4,1);
const IPAddress _gateway(192,168,4,1);
const IPAddress _subnet(255,255,255,0);

AsyncWebServer server(HTTP_PORT);

AsyncEventSource sensor_emitter("/robot/sensors/events");
AsyncEventSource malfunction_emitter("/robot/malfunctions/events");

float _battery_voltage;

Ticker battery_timer;

int _throttle = 850;
uint8_t _direction = FORWARD;
bool _stopped = true;

MPU6050 mpu(MPU_ADDRESS);

Quaternion _q;
VectorFloat _gravity;
float _heading, _pitch, _roll;
float _heading_lock, _temperature;
uint8_t _dmp_fifo_buffer[64];
uint16_t _dmp_packet_size;
volatile bool _dmp_ready = false;
void ICACHE_RAM_ATTR dmpDataReady();

Ticker gravity_timer, orientation_timer;
Ticker temperature_timer;

float _rotator_prev_delta, _rotator_delta_integral;
float _stabilizer_prev_delta, _stabilizer_delta_integral;

Ticker rotator_timer, stabilizer_timer;
Ticker rollover_timer;

VL53L1X lidar;

float _scan_angles[NUM_SCANS];
int _distances_to_object[NUM_SCANS];
uint8_t _scan_count;

Ticker scan_timer, collision_timer;

float _mover_prediction;
long _mover_start_timestamp, _mover_end_timestamp;
float _mover_features[4];
float _mover_weights[4], _mover_bias;
float _mover_weight_velocities[4], _mover_bias_velocity;
float _mover_weight_norms[4], _mover_bias_norm;

Ticker mover_timer;

uint8_t _explorer_step;

Ticker explore_timer;

/**
 * Set up the serial interface.
 */
void setupSerial() {
  Serial.begin(SERIAL_BAUD);

  Serial.println("Serial port enabled");
}

/**
 * Setup the beeper.
 */
void setupBeeper() {
  pinMode(BEEPER_PIN, OUTPUT);

  Serial.println("Beeper enabled");
}

/**
 * Set up the central processing unit.
 */
void setupCPU() {
  Serial.print("CPU speed: ");
  Serial.print(ESP.getCpuFreqMHz());
  Serial.println("mhz");
}

/**
 * Setup the flash ROM.
 */
void setupROM() {   
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
void setupSPIFFS() {
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
void setupAccessPoint() {  
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
void setupHttpServer() {
  server.serveStatic("/ui", SPIFFS, "/app.html");
  server.serveStatic("/ui/control", SPIFFS, "/app.html");
  server.serveStatic("/manifest.json", SPIFFS, "/manifest.json");
  server.serveStatic("/app.js", SPIFFS, "/app.js");
  server.serveStatic("/sw.js", SPIFFS, "/sw.js");
  server.serveStatic("/app.css", SPIFFS, "/app.css");
  server.serveStatic("/images/app-icon-small.png", SPIFFS, "/images/app-icon-small.png");
  server.serveStatic("/images/app-icon-large.png", SPIFFS, "/images/app-icon-large.png");
  server.serveStatic("/fonts/Roboto-300.woff2", SPIFFS, "/fonts/Roboto-300.woff2");
  server.serveStatic("/fonts/Roboto-300.woff", SPIFFS, "/fonts/Roboto-300.woff");
  server.serveStatic("/fonts/Roboto-regular.woff2", SPIFFS, "/fonts/Roboto-regular.woff2");
  server.serveStatic("/fonts/Roboto-regular.woff", SPIFFS, "/fonts/Roboto-regular.woff");
  server.serveStatic("/fonts/Roboto-500.woff2", SPIFFS, "/fonts/Roboto-500.woff2");
  server.serveStatic("/fonts/Roboto-500.woff", SPIFFS, "/fonts/Roboto-500.woff");
  server.serveStatic("/fonts/fa-solid-900.woff2", SPIFFS, "/fonts/fa-solid-900.woff2");
  server.serveStatic("/fonts/fa-solid-900.woff", SPIFFS, "/fonts/fa-solid-900.woff");
  server.serveStatic("/sounds/plucky.ogg", SPIFFS, "/sounds/plucky.ogg");

  server.on("/robot", HTTP_GET, handleGetRobot);
  server.on("/robot/motors", HTTP_DELETE, handleStop);
  server.on("/robot/features/beeper", HTTP_PUT, handleBeep);
  server.on("/robot/features/autonomy", HTTP_PUT, handleExplore);
  server.on("/robot/features/autonomy", HTTP_DELETE, handleStop);

  server.addHandler(new AsyncCallbackJsonWebHandler("/robot/motors/direction", handleChangeDirection));
  server.addHandler(new AsyncCallbackJsonWebHandler("/robot/motors/throttle", handleSetThrottlePercentage));
  server.addHandler(new AsyncCallbackJsonWebHandler("/robot/rotator/left", handleRotateLeft));
  server.addHandler(new AsyncCallbackJsonWebHandler("/robot/rotator/right", handleRotateRight));

  server.addHandler(&sensor_emitter);
  server.addHandler(&malfunction_emitter);

  server.onNotFound(handleNotFound);

  server.begin();

  Serial.print("HTTP server listening on port ");
  Serial.println(HTTP_PORT);
}

/**
 * Set up the battery voltmeter.
 */
void setupBattery() {
  pinMode(BATTERY_PIN, INPUT);
  
  battery_timer.attach_ms(BATTERY_SAMPLE_RATE, updateBattery);

  Serial.println("Battery voltmeter enabled");
}

/**
 * Set up the motors.
 */
void setupMotors() {
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
void setupI2C() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK);

  Serial.println("I2C bus initialized");
}

/**
 * Set up and calibrate the MPU accelerometer, gyroscope, and temperature sensor.
 */
void setupMPU() {
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
  mpu.setSleepEnabled(false);

  Serial.print("Calibrating gyroscope ");
  mpu.CalibrateGyro(GYRO_CALIBRATION_LOOPS);
  Serial.println(" done");

  Serial.print("Calibrating accelerometer ");
  mpu.CalibrateAccel(ACCEL_CALIBRATION_LOOPS);
  Serial.println(" done");

  mpu.dmpInitialize();

  mpu.setDMPEnabled(true);

  _dmp_packet_size = mpu.dmpGetFIFOPacketSize();

  gravity_timer.attach_ms(GRAVITY_SAMPLE_RATE, updateGravity);
  orientation_timer.attach_ms(ORIENTATION_SAMPLE_RATE, updateOrientation);
  temperature_timer.attach_ms(TEMPERATURE_SAMPLE_RATE, updateTemperature);

  Serial.println("DMP initialized");
}

/**
 * Set up the lidar distance sensor.
 */
void setupLidar() {
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
}

/**
 * Set up the random number generator.
 */
void setupRandom() {
  unsigned long seed = analogRead(BATTERY_PIN) + abs(mpu.getTemperature());
  
  randomSeed(seed);
  
  Serial.println("Seeded random number generator");
}

/**
 * Set up the mover model.
 */
void setupMover() {
  for (uint8_t i = 0; i < 4; i++) {
    _mover_weights[i] = random(-MAX_RAND_INT, MAX_RAND_INT) / (float) MAX_RAND_INT;
  }
  
  Serial.println("Mover model initialized");
}

/**
 * Bootstrap the robot.
 */
void setup() {    
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
void loop() {
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

      mpu.dmpGetQuaternion(&_q, _dmp_fifo_buffer);
    }

    _dmp_ready = false;
  }
}

/**
 * Handle a get robot request.
 */
int handleGetRobot(AsyncWebServerRequest *request) {
  StaticJsonDocument<512> doc;
  char buffer[512];

  JsonObject robot = doc.createNestedObject("robot");

  JsonObject motors = robot.createNestedObject("motors");

  motors["direction"] = _direction;
  motors["throttle"] = throttlePercentage();
  motors["stopped"] = _stopped;

  JsonObject sensors = robot.createNestedObject("sensors");

  JsonObject battery = sensors.createNestedObject("battery");

  battery["voltage"] = _battery_voltage;
  battery["percentage"] = batteryPercentage();
  
  sensors["temperature"] = _temperature;

  JsonObject features = robot.createNestedObject("features");

  features["autonomy"] = isAutonomous();
  
  serializeJson(doc, buffer);

  request->send(HTTP_OK, "application/json", buffer);
}

/**
 * Handle a change direction request.
 */
void handleChangeDirection(AsyncWebServerRequest *request, JsonVariant &json) {
  const JsonObject& doc = json.as<JsonObject>();

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
void handleStop(AsyncWebServerRequest *request) {
  stop();

  request->send(HTTP_OK);
}

/**
 * Handle the stop movement request.
 */
void handleExplore(AsyncWebServerRequest *request) {
  explore();

  request->send(HTTP_OK);
}

/**
 * Handle a beep request.
 */
void handleBeep(AsyncWebServerRequest *request) {
  beep(2);

  request->send(HTTP_OK);
}

/**
 * Handle a set throttle position request.
 */
void handleSetThrottlePercentage(AsyncWebServerRequest *request, JsonVariant &json) {
  const JsonObject& doc = json.as<JsonObject>();

  if (!doc.containsKey("throttle")) {
    request->send(HTTP_UNPROCESSABLE_ENTITY);

    return;
  }

  int throttle = doc["throttle"];

  setThrottlePercentage(throttle);

  request->send(HTTP_OK);
}

/**
 * Handle the rotate left request.
 */
void handleRotateLeft(AsyncWebServerRequest *request, JsonVariant &json) {
  const JsonObject& doc = json.as<JsonObject>();

  if (!doc.containsKey("radians")) {
    request->send(HTTP_UNPROCESSABLE_ENTITY);

    return;
  }

  float radians = doc["radians"];
      
  rotateLeft(radians);

  request->send(HTTP_OK);
}

/**
 * Handle the rotate right request.
 */
void handleRotateRight(AsyncWebServerRequest *request, JsonVariant &json) {
  const JsonObject& doc = json.as<JsonObject>();

  if (!doc.containsKey("radians")) {
    request->send(HTTP_UNPROCESSABLE_ENTITY);

    return;
  }

  float radians = doc["radians"];
      
  rotateRight(radians);
  
  request->send(HTTP_OK);
}

/**
 * Catch all route responds with 404.
 */
int handleNotFound(AsyncWebServerRequest *request) {
  request->send(HTTP_NOT_FOUND);
}

/**
 * Move forward.
 */
void forward() {  
  _direction = FORWARD;

  enableCollisionDetection();
  enableStabilizer();

  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

  go();
}

/**
 * Turn left.
 */
void left() {    
  _direction = LEFT;

  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);

  go();
}

/**
 * Turn right.
 */
void right() {  
  _direction = RIGHT;

  digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

  go();
}

/**
 * Move in reverse.
 */
void reverse() {
  _direction = REVERSE;

  enableStabilizer();

  digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
  digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);

  go();
}

/**
 * Engage the motors.
 */
void go() {    
  _stopped = false;

  enableRolloverDetection();

  analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
  analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);
}

/**
 * Apply the brake to the motors.
 */
void brake() {
  analogWrite(MOTOR_A_THROTTLE_PIN, 0);
  analogWrite(MOTOR_B_THROTTLE_PIN, 0);
}

/**
 * Disengage the motors and related timers.
 */
void stop() {
  brake();

  disableCollisionDetection();
  disableStabilizer();
  disableRolloverDetection();

  _stopped = true;
  
  explore_timer.detach();
}

/**
 * Returns the battery voltage as a percentage.
 */
int batteryPercentage() {
  return map(_battery_voltage, MIN_VOLTAGE, MAX_VOLTAGE, 0, 100);
}

/**
 * Returns the throttle position as a percentage.
 */
int throttlePercentage() {
  return map(_throttle, MIN_THROTTLE, MAX_THROTTLE, 0, 100);
}

/**
 * Set the throttle percentage of the motor controller.
 */
void setThrottlePercentage(int throttle) {
  _throttle = map(throttle, 0, 100, MIN_THROTTLE, MAX_THROTTLE);

  if (!_stopped) {
    analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
    analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);
  }
}

/**
 * Rotate the vehicle to the left.
 */
void rotateLeft(float radians) {
  _direction = LEFT;

  rotate(-radians);

  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);
}

/**
 * Rotate the vehicle to the right.
 */
void rotateRight(float radians) {  
  _direction = RIGHT;

  rotate(radians);

  digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);
}

/**
 * Rotate the vehicle.
 */
void rotate(float radians) {
  float target = constrain(radians, -M_PI, M_PI);
    
  _heading_lock = calculateTargetAngle(_heading, target);

  _rotator_delta_integral = 0.0;
  _rotator_prev_delta = 0.0;

  rotator_timer.attach_ms(ROTATOR_SAMPLE_RATE, rotator);
}

/**
 * Control loop to rotate the vehicle by actuating the motors.
 */
void rotator() {
  float delta = calculateAngleDelta(_heading, _heading_lock);

  _rotator_delta_integral += 0.5 * (delta + _rotator_prev_delta);

  _rotator_delta_integral = constrain(_rotator_delta_integral, -M_PI, M_PI);
      
  float derivative = delta - _rotator_prev_delta;

  float theta = ROTATOR_P_GAIN * delta
    + ROTATOR_I_GAIN * _rotator_delta_integral
    + ROTATOR_D_GAIN * derivative;

  float alpha = min(fabs(theta) / M_PI, 1.0);

  unsigned int rotator_throttle = max(MIN_THROTTLE, (int) round(alpha * _throttle));

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
void explore() {
  stop();

  _explorer_step = 0;
  
  explore_timer.attach_ms(EXPLORE_SAMPLE_RATE, explorer);
}

/**
 * Is the robot acting in autonomous mode?
 */
bool isAutonomous() {
  return explore_timer.active();
}

/**
 * Control loop to explore the environment.
 */
void explorer() {
  bool ready = !scan_timer.active() && !mover_timer.active() && !rotator_timer.active();
  
  if (ready) {
    switch (_explorer_step) {
      case 0: {
        scanEnvironment();
        
        _explorer_step = 1;
      }
      
      break;

      case 1: {
        choosePath();

        _explorer_step = 2;
      }
      
      break;
        
      case 2: {
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
void scanEnvironment() {
  rotateRight(0.5 * SCAN_WINDOW);
    
  _scan_count = 0;

  scan_timer.attach_ms(SCAN_SAMPLE_RATE, scanner);
}

/**
 * Control loop to scan the environment for objects.
 */
void scanner() {
  if (!rotator_timer.active()) {
    _distances_to_object[_scan_count] = distanceToObject();
    
    _scan_angles[_scan_count] = _heading;
   
    ++_scan_count;
    
    if (_scan_count < NUM_SCANS) {
      rotateLeft(SCAN_WINDOW / (NUM_SCANS - 1));
    } else {
      scan_timer.detach();
    }
  }
}

/**
 * Choose a path to traverse.
 */
void choosePath() {
  long weights[NUM_SCANS];
  
  for (uint8_t i = 0; i < NUM_SCANS; i++) {
    weights[i] = round(pow(_distances_to_object[i], PATH_AFFINITY));
  }
  
  long total = 0;

  for (uint8_t i = 0; i < NUM_SCANS; i++) {
    total += weights[i];
  }
 
  long rho = random(0, total);

  float delta;
  
  for (uint8_t i = 0; i < NUM_SCANS; i++) {
    rho -= weights[i];

    if (rho <= 0) {
      delta = calculateAngleDelta(_heading, _scan_angles[i]);
            
      break;
    }
  }

  rotate(delta);
}

/**
 * Move the robot forward.
 */
void move() {
  _mover_features[0] = throttlePercentage() / 100.0;
  _mover_features[1] = batteryPercentage() / 100.0;
  _mover_features[2] = _pitch / HALF_PI;
  _mover_features[3] = distanceToObject() / (float) LIDAR_MAX_RANGE;

  float delta = 0.0;

  for (uint8_t i = 0; i < 4; i++) {
    delta += _mover_features[i] * _mover_weights[i];
  }

  delta += _mover_bias;

  long now = millis();

  _mover_prediction = delta;

  _mover_start_timestamp = now;

  long max_overshoot = round(MOVER_MAX_OVERSHOOT * fabs(delta));

  long overshoot = random(0, max_overshoot);

  long burn_time = round(fmax(0.0, delta + overshoot));
  
  _mover_end_timestamp = now + burn_time;

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
void mover() {
  long now = millis();

  if (now > _mover_end_timestamp || distanceToObject() < COLLISION_THRESHOLD) {
    brake();

    mover_timer.detach();

    float delta = now - _mover_start_timestamp;

    float dydl = _mover_prediction - delta;

    float dydw;

    for (uint8_t i = 0; i < 4; i++) {
      dydw = dydl * _mover_features[i];
      
      dydw += MOVER_ALPHA * _mover_weights[i];

      _mover_weight_velocities[i] = dydw * MOVER_MOMENTUM_DECAY
        + _mover_weight_velocities[i] * (1.0 - MOVER_MOMENTUM_DECAY);
      
      _mover_weight_norms[i] = sq(dydw) * MOVER_NORM_DECAY
        + _mover_weight_norms[i] * (1.0 - MOVER_NORM_DECAY);
      
      _mover_weights[i] -= (MOVER_LEARNING_RATE * _mover_weight_velocities[i])
        / sqrt(_mover_weight_norms[i]);
    }

    _mover_bias_velocity = dydl * MOVER_MOMENTUM_DECAY
      + _mover_bias_velocity * (1.0 - MOVER_MOMENTUM_DECAY);

    _mover_bias_norm = sq(dydl) * MOVER_NORM_DECAY
      + _mover_bias_norm * (1.0 - MOVER_NORM_DECAY);

    _mover_bias -= (MOVER_LEARNING_RATE * _mover_bias_velocity)
      / sqrt(_mover_bias_norm);
  }
}

/**
 * Enable linear movement stabilizer.
 */
void enableStabilizer() {
  _heading_lock = _heading;
  
  _stabilizer_delta_integral = 0.0;
  _stabilizer_prev_delta = 0.0;

  stabilizer_timer.attach_ms(STABILIZER_SAMPLE_RATE, stabilize);
}

/**
 * Disable the linear stabilizer.
 */
void disableStabilizer() {
  stabilizer_timer.detach();
}

/**
 * Stabilize linear movement using orientation feedback from the gyroscope.
 */
void stabilize() {
  float delta = calculateAngleDelta(_heading, _heading_lock);

  _stabilizer_delta_integral += 0.5 * (delta + _stabilizer_prev_delta);

  _stabilizer_delta_integral = constrain(_stabilizer_delta_integral, -M_PI, M_PI);
      
  float derivative = delta - _stabilizer_prev_delta;

  float theta = STABILIZER_P_GAIN * delta
    + STABILIZER_I_GAIN * _stabilizer_delta_integral
    + STABILIZER_D_GAIN * derivative;

  float beta = min(fabs(theta) / M_PI, 1.0);

  int backoff_throttle = max(MIN_THROTTLE, (int) round((1.0 - beta) * _throttle));

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
void enableRolloverDetection() {
  rollover_timer.attach_ms(ROLLOVER_SAMPLE_RATE, detectRollover);
}

/**
 * Disable rollover detection.
 */
void disableRolloverDetection() {
  rollover_timer.detach();
}

/**
 * Detect if the vehicle has rolled over.
 */
void detectRollover() {
  if (fabs(_pitch) > ROLLOVER_THRESHOLD || fabs(_roll) > ROLLOVER_THRESHOLD) {
    stop();

    beep(5);

    malfunction_emitter.send("", "rollover-detected");
  }
}

/**
 * Enable collision detection.
 */
void enableCollisionDetection() {
  collision_timer.attach_ms(COLLISION_SAMPLE_RATE, detectCollision);
}

/**
 * Disable collision detection.
 */
void disableCollisionDetection() {
  collision_timer.detach();
}

/**
 * Detect if collision with an object is iminent and stop the vehicle.
 */
void detectCollision() {
  if (distanceToObject() < COLLISION_THRESHOLD) {
    brake();

    malfunction_emitter.send("", "collision-detected");
  }
}

/**
 * Update the battery voltage.
 */
void updateBattery() {
  StaticJsonDocument<64> doc;
  char buffer[64];
      
  unsigned int raw = analogRead(BATTERY_PIN);

  _battery_voltage = (raw / VOLTMETER_RESOLUTION) * MAX_VOLTAGE;

  if (_battery_voltage > 1.0 && _battery_voltage < MIN_VOLTAGE) {
    stop();

    if (malfunction_emitter.count() > 0) {
      doc["voltage"] = _battery_voltage;
  
      serializeJson(doc, buffer);

      malfunction_emitter.send(buffer, "battery-undervoltage");
    }

    panicNow();
  } else {
    if (sensor_emitter.count() > 0) {
      doc["voltage"] = _battery_voltage;
      doc["percentage"] = batteryPercentage();
  
      serializeJson(doc, buffer);
    
      sensor_emitter.send(buffer, "battery-voltage-updated");
    }
  }
}

/**
 * Update the temperature.
 */
void updateTemperature() {
  StaticJsonDocument<64> doc;
  char buffer[64];
    
  long raw = mpu.getTemperature();
  
  _temperature = raw / TEMPERATURE_SENSITIVITY + TEMPERATURE_CONSTANT;

  if (sensor_emitter.count() > 0) {      
    doc["temperature"] = _temperature;
  
    serializeJson(doc, buffer);

    sensor_emitter.send(buffer, "temperature-updated");
  }
}

/**
 * Beep the beeper.
 */
void beep(uint8_t count) {
  digitalWrite(BEEPER_PIN, HIGH);

  --count;

  beep_timer.once_ms(BEEP_DURATION, silenceBeeper, count);
}

/**
 * Silence the beeper.
 */
void silenceBeeper(uint8_t remaining) {
  digitalWrite(BEEPER_PIN, LOW);

  if (remaining > 0) {
    beep_timer.once_ms(BEEP_DELAY, beep, remaining);
  }
}

/**
 * Update the gravity vector from the raw quaternion.
 */
void updateGravity() {
  _gravity.x = -2.0 * (_q.x * _q.z - _q.w * _q.y);
  _gravity.y = 2.0 * (_q.w * _q.x + _q.y * _q.z);
  _gravity.z = -(sq(_q.w) - sq(_q.x) -sq(_q.y) + sq(_q.z));
}

/**
 * Calculate the current heading, pitch, and roll from the raw quaternion and gravity vector.
 */
void updateOrientation() {
  _heading = atan2(2.0 * -_q.x * _q.y - 2.0 * _q.w * -_q.z, 2.0 * sq(_q.w) + 2.0 * sq(_q.x) - 1.0);
  
  _pitch = atan2(_gravity.x, sqrt(sq(_gravity.y) + sq(_gravity.z)));
  
  _roll = atan2(_gravity.y, _gravity.z);
}

/**
 * Return the distance to object using the narrow beam.
 */
int distanceToObject() {
  int distance;
    
  switch (lidar.ranging_data.range_status) {
    case LIDAR_RANGE_VALID:
      distance = lidar.ranging_data.range_mm;
        
      break;

    case LIDAR_NOISY_SIGNAL:
      distance = round(0.5 * lidar.ranging_data.range_mm);
        
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
 * Calculate the final angle of a rotation from a starting point with the addition of delta in radians.
 */
float calculateTargetAngle(float start, float delta) {
  float target = start + delta;
  
  if (target > M_PI) {
    target -= TWO_PI;
  } else if (target < -M_PI) {
    target += TWO_PI;
  }

  return target;
}

/**
 * Calculate the difference between a target and the current heading.
 */
float calculateAngleDelta(float start, float target) {
  float delta = target - start;

  if (fabs(delta) > M_PI) {
    if (start > 0.0) {
      delta = target - (start - TWO_PI);
    } else {
      delta = target - (start + TWO_PI);
    }
  }

  return delta;
}

/**
 * What to do when we don't know what to do.
 */
void panicNow() {
  beep(4);

  delay(4 * (BEEP_DURATION + BEEP_DELAY));

  ESP.deepSleep(0);
}

/**
 * MPU interrupt callback to signal DMP data is ready in the buffer.
 */
void dmpDataReady() {
  _dmp_ready = true;
}

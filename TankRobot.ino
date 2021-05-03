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
#define GYRO_CALIBRATION_LOOPS 6
#define GRAVITY_SAMPLE_RATE 10
#define ORIENTATION_SAMPLE_RATE GRAVITY_SAMPLE_RATE
#define ACCEL_CALIBRATION_LOOPS 6
#define ACCEL_SAMPLE_RATE 10
#define ACCEL_LSB_PER_G 16384.0
#define ACCEL_HIGH_PASS_COEF 0.7
#define VELOCITY_SAMPLE_RATE ACCEL_SAMPLE_RATE
#define DISPLACEMENT_SAMPLE_RATE VELOCITY_SAMPLE_RATE
#define TEMPERATURE_SAMPLE_RATE 1000
#define TEMPERATURE_SENSITIVITY 340.0
#define TEMPERATURE_CONSTANT 36.53

#define ROTATOR_SAMPLE_RATE ORIENTATION_SAMPLE_RATE
#define ROTATOR_THRESHOLD 3.0 * DEG_TO_RAD
#define ROTATOR_P_GAIN 2.0
#define ROTATOR_I_GAIN 0.7
#define ROTATOR_D_GAIN 1.4

#define MOVER_SAMPLE_RATE DISPLACEMENT_SAMPLE_RATE 

#define STABILIZER_SAMPLE_RATE 50
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

#define SCAN_SAMPLE_RATE 250
#define SCAN_WINDOW M_PI
#define NUM_SCANS 5

#define COLLISION_SAMPLE_RATE LIDAR_SAMPLE_RATE
#define COLLISION_THRESHOLD 300

#define ROLLOVER_SAMPLE_RATE 500
#define ROLLOVER_THRESHOLD HALF_PI

#define BEEPER_PIN D7
#define BEEP_DURATION 130
#define BEEP_DELAY 40

#define EXPLORE_SAMPLE_RATE 500
#define STATUS_SAMPLE_RATE 3000

#define EPSILON 1e-8

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
VectorInt16 _aa;
VectorFloat _gravity, _aa_real;
VectorFloat _acceleration, _velocity, _displacement;
VectorFloat _prev_aa_real, _prev_acceleration, _prev_velocity;
float _yaw, _pitch, _roll;
float _yaw_lock, _temperature;
uint8_t _dmp_fifo_buffer[64];
uint16_t _dmp_packet_size;
void ICACHE_RAM_ATTR dmpDataReady();
volatile bool _dmp_ready = false;

Ticker gravity_timer, orientation_timer;
Ticker acceleration_timer, velocity_timer, displacement_timer;
Ticker temperature_timer, rollover_timer;

Ticker mover_timer;

float _rotator_prev_delta, _rotator_delta_integral;

Ticker rotator_timer;

float _stabilizer_prev_delta, _stabilizer_delta_integral;

Ticker stabilizer_timer;

VL53L1X lidar;

float _scan_angles[NUM_SCANS];
uint16_t _distances_to_object[NUM_SCANS];
uint8_t _scan_count;

Ticker scan_timer, collision_timer;

uint8_t _explorer_step;

Ticker explore_timer;

Ticker beep_timer;

/**
 * Bootstrap routine to setup the robot.
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

  Serial.println("Ready!");
}

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
  Serial.print("CPU clock: ");
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

  Serial.print("ROM size: ");
  Serial.print(ESP.getFlashChipRealSize() / 1024);
  Serial.println("kb");

  Serial.print("Free space: ");
  Serial.print(ESP.getFreeSketchSpace() / 1024.0, 1);
  Serial.println("kb");

  Serial.print("ROM speed: ");
  Serial.print(ESP.getFlashChipSpeed() / 1000000);
  Serial.println("mhz");
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
  server.addHandler(new AsyncCallbackJsonWebHandler("/robot/motors/throttle", handleSetThrottle));
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

  brake();

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

  pinMode(MPU_INTERRUPT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);

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
  randomSeed(analogRead(BATTERY_PIN));
  
  Serial.println("Seeded random number generator");
}

/**
 * Main event loop.
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

      mpu.dmpGetAccel(&_aa, _dmp_fifo_buffer);
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
  motors["throttle"] = 100.0 * (_throttle / MAX_THROTTLE);
  motors["stopped"] = _stopped;

  JsonObject sensors = robot.createNestedObject("sensors");

  sensors["voltage"] = _battery_voltage;
  sensors["temperature"] = _temperature;

  JsonObject features = robot.createNestedObject("features");
  
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
void handleSetThrottle(AsyncWebServerRequest *request, JsonVariant &json) {
  const JsonObject& doc = json.as<JsonObject>();

  if (!doc.containsKey("throttle")) {
    request->send(HTTP_UNPROCESSABLE_ENTITY);

    return;
  }

  int throttle = (int) doc["throttle"];

  setThrottle(throttle);

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
  _yaw_lock = _yaw;
  
  _stabilizer_delta_integral = 0.0;
  _stabilizer_prev_delta = 0.0;

  stabilizer_timer.attach_ms(STABILIZER_SAMPLE_RATE, stabilize);
  collision_timer.attach_ms(COLLISION_SAMPLE_RATE, detectCollision);

  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

  go();
}

/**
 * Turn in the left direction.
 */
void left() {    
  _direction = LEFT;

  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);

  go();
}

/**
 * Turn in the right direction.
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
  _yaw_lock = _yaw;
  
  _stabilizer_delta_integral = 0.0;
  _stabilizer_prev_delta = 0.0;

  stabilizer_timer.attach_ms(STABILIZER_SAMPLE_RATE, stabilize);

  digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
  digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);

  go();
}

/**
 * Engage the motors.
 */
void go() {    
  _stopped = false;

  rollover_timer.attach_ms(ROLLOVER_SAMPLE_RATE, detectRollover);

  analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
  analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);
}

/**
 * Disengage the motors and related timers.
 */
void stop() {
  brake();

  _stopped = true;
    
  mover_timer.detach();
  stabilizer_timer.detach();
  rotator_timer.detach();
  collision_timer.detach();
  rollover_timer.detach();
  explore_timer.detach();
}

/**
 * Apply the brake to the motors.
 */
void brake() {
  analogWrite(MOTOR_A_THROTTLE_PIN, 0);
  analogWrite(MOTOR_B_THROTTLE_PIN, 0);
}

/**
 * Set the throttle position of the motor controller.
 */
void setThrottle(int throttle) {
  _throttle = map(throttle, 0, 100, MIN_THROTTLE, MAX_THROTTLE);

  if (!_stopped) {
    analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
    analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);
  }
}

/**
 * Rotate the vehicle x radians to the left.
 */
void rotateLeft(float radians) {
  radians = constrain(radians, 0, TWO_PI);
  
  _yaw_lock = calculateTargetAngle(_yaw, -radians);
  
  _direction = LEFT;

  _rotator_delta_integral = 0.0;
  _rotator_prev_delta = 0.0;

  rotator_timer.attach_ms(ROTATOR_SAMPLE_RATE, rotator);

  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);
}

/**
 * Rotate the vehicle by x radians to the right.
 */
void rotateRight(float radians) {
  radians = constrain(radians, 0, TWO_PI);

  _yaw_lock = calculateTargetAngle(_yaw, radians);
   
  _direction = RIGHT;

  _rotator_delta_integral = 0.0;
  _rotator_prev_delta = 0.0;

  rotator_timer.attach_ms(ROTATOR_SAMPLE_RATE, rotator);

  digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);
}

/**
 * Handle the rotate left request.
 */
void handleRotateLeft(AsyncWebServerRequest *request, JsonVariant &json) {
  const JsonObject& doc = json.as<JsonObject>();

  if (doc.containsKey("radians")) {
    float radians = (float) doc["radians"];
      
    rotateLeft(radians);
  } else {
    rotateLeft(HALF_PI);
  }

  request->send(HTTP_OK);
}

/**
 * Handle the rotate right request.
 */
void handleRotateRight(AsyncWebServerRequest *request, JsonVariant &json) {
  const JsonObject& doc = json.as<JsonObject>();

  if (doc.containsKey("radians")) {
    float radians = (float) doc["radians"];
      
    rotateRight(radians);
  } else {
    rotateRight(HALF_PI);
  }
  
  request->send(HTTP_OK);
}

/**
 * Control loop to rotate the vehicle by actuating the motors.
 */
void rotator() {
  float delta = calculateYawDelta(_yaw, _yaw_lock);

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
 * Control loop to explore the environment.
 */
void explorer() {
  if (!scan_timer.active() && !mover_timer.active() && !rotator_timer.active()) {
    switch (_explorer_step) {
      case 0: {
        scan();
        
        _explorer_step = 1;
      }
      
      break;

      case 1: {
        uint8_t index = randomWeightedIndex(_distances_to_object, NUM_SCANS);

        float delta = calculateYawDelta(_yaw, _scan_angles[index]);

        if (delta < 0.0) {
          rotateLeft(fabs(delta));
        } else {
          rotateRight(delta);
        }

        _explorer_step = 2;
      }
      
      break;
        
      case 2: {
        move(2000);

        _explorer_step = 0;
      }

      break;
    }
  }
}

/**
 * Move the robot forward.
 */
void move(unsigned long duration) {
  forward();

  mover_timer.once_ms(duration, stopMoving);
}

/**
 * Stop moving.
 */
void stopMoving() {
  brake();

  stabilizer_timer.detach();
  collision_timer.detach();
  mover_timer.detach();
}

/**
 * Scan the environment.
 */
void scan() {
  rotateRight(0.5 * SCAN_WINDOW);
    
  _scan_count = 0;

  scan_timer.attach_ms(SCAN_SAMPLE_RATE, scanLoop);
}

/**
 * Control loop to scan the environment for objects.
 */
void scanLoop() {
  if (!rotator_timer.active()) {
    int distance;
    
    switch (lidar.ranging_data.range_status) {
      case LIDAR_RANGE_VALID:
        distance = lidar.ranging_data.range_mm;
        
        break;

      case LIDAR_NOISY_SIGNAL:
        distance = 0.5 * lidar.ranging_data.range_mm;
        
        break;

      case LIDAR_SIGNAL_FAILURE:
      case LIDAR_PHASE_OUT_OF_BOUNDS:
        distance = LIDAR_MAX_RANGE;
        
        break;
    }

    distance += LIDAR_SENSOR_OFFSET;

    _distances_to_object[_scan_count] = max(0, distance);
    _scan_angles[_scan_count] = _yaw;
   
    _scan_count++;
    
    if (_scan_count < NUM_SCANS) {
      rotateLeft(SCAN_WINDOW / (NUM_SCANS - 1));
    } else {
      scan_timer.detach();
    }
  }
}

/**
 * Stabilize linear movement using orientation feedback from the gyroscope.
 */
void stabilize() {
  float delta = calculateYawDelta(_yaw, _yaw_lock);

  _stabilizer_delta_integral += 0.5 * (delta + _stabilizer_prev_delta);

  _stabilizer_delta_integral = constrain(_stabilizer_delta_integral, -M_PI, M_PI);
      
  float derivative = delta - _stabilizer_prev_delta;

  float theta = STABILIZER_P_GAIN * delta
    + STABILIZER_I_GAIN * _stabilizer_delta_integral
    + STABILIZER_D_GAIN * derivative;

  float beta = min(fabs(theta) / M_PI, 1.0);

  unsigned int backoff_throttle = max(MIN_THROTTLE, (int) round((1.0 - beta) * _throttle));

  if (delta < 0.0) {
    switch (_direction) {
      case FORWARD:
        analogWrite(MOTOR_B_THROTTLE_PIN, backoff_throttle);
        
        break;
    
      case REVERSE:
        analogWrite(MOTOR_A_THROTTLE_PIN, backoff_throttle);
        
        break;
    }
  } else {
    switch (_direction) {
      case FORWARD:
        analogWrite(MOTOR_A_THROTTLE_PIN, backoff_throttle);
        
        break;
    
      case REVERSE:
        analogWrite(MOTOR_B_THROTTLE_PIN, backoff_throttle);
            
        break;
    }
  }
}

/**
 * MPU interrupt callback to signal DMP data is ready in the buffer.
 */
void dmpDataReady() {
  _dmp_ready = true;
}

/**
 * Update the gravity vector from the raw quaternion.
 */
void updateGravity() {
  _gravity.x = -2.0 * (_q.x * _q.z - _q.w * _q.y);
  _gravity.y = 2.0 * (_q.w * _q.x + _q.y * _q.z);
  _gravity.z = -(pow(_q.w, 2) - pow(_q.x, 2) - pow(_q.y, 2) + pow(_q.z, 2));
}

/**
 * Calculate the current yaw, pitch, and roll from the raw quaternion and gravity vector.
 */
void updateOrientation() {
  _yaw = atan2(2.0 * -_q.x * _q.y - 2.0 * _q.w * -_q.z, 2.0 * pow(_q.w, 2) + 2.0 * pow(_q.x, 2) - 1.0);
  
  _pitch = atan2(_gravity.x, sqrt(pow(_gravity.y, 2) + pow(_gravity.z, 2)));
  
  _roll = atan2(_gravity.y, _gravity.z);
}

/**
 * Update the current linear acceleration of the vehicle with gravity vector removed.
 */
void updateAcceleration() {
  _aa_real.x = (_aa.x / ACCEL_LSB_PER_G) - _gravity.x;
  _aa_real.y = (_aa.y / ACCEL_LSB_PER_G) - _gravity.y;
  _aa_real.z = (_aa.z / ACCEL_LSB_PER_G) - _gravity.z;

  _acceleration.x = ACCEL_HIGH_PASS_COEF * (_acceleration.x + _aa_real.x - _prev_aa_real.x);
  _acceleration.y = ACCEL_HIGH_PASS_COEF * (_acceleration.y + _aa_real.y - _prev_aa_real.y);
  _acceleration.z = ACCEL_HIGH_PASS_COEF * (_acceleration.y + _aa_real.z - _prev_aa_real.z);

  _prev_aa_real = _aa_real;
}

/**
 * Update the velocity of the vehicle by integrating acceleration.
 */
void updateVelocity() {
  _velocity.x += 0.5 * (_acceleration.x + _prev_acceleration.x);
  _velocity.y += 0.5 * (_acceleration.y + _prev_acceleration.y);
  _velocity.z += 0.5 * (_acceleration.z + _prev_acceleration.z);

  _prev_acceleration = _acceleration;
}

/**
 * Update the displacement of the vehicle by integrating velocity.
 */
void updateDisplacement() {
  _displacement.x += 0.5 * (_velocity.x + _prev_velocity.x);
  _displacement.y += 0.5 * (_velocity.y + _prev_velocity.y);
  _displacement.z += 0.5 * (_velocity.z + _prev_velocity.z);

  _prev_velocity = _velocity;
}

/**
 * Update the temperature.
 */
void updateTemperature() {
  long raw = mpu.getTemperature();
  
  _temperature = raw / TEMPERATURE_SENSITIVITY + TEMPERATURE_CONSTANT;

  if (sensor_emitter.count() > 0) {
    StaticJsonDocument<64> doc;
    char buffer[64];
      
    doc["temperature"] = _temperature;
  
    serializeJson(doc, buffer);

    sensor_emitter.send(buffer, "temperature-updated");
  }
}

/**
 * Detect if collision with an object is iminent and stop the vehicle.
 */
void detectCollision() {
  uint16_t distance = LIDAR_SENSOR_OFFSET + lidar.ranging_data.range_mm;
  
  if (distance < COLLISION_THRESHOLD) {
    brake();

    malfunction_emitter.send("", "collision-detected");
  }
}

/**
 * Detect if the vehicle has rolled over.
 */
void detectRollover() {
  if (fabs(_pitch) > ROLLOVER_THRESHOLD || fabs(_roll) > ROLLOVER_THRESHOLD) {
    stop();

    beep(4);

    malfunction_emitter.send("", "rollover-detected");
  }
}

/**
 * Update the battery voltage.
 */
void updateBattery() {
  unsigned int raw = analogRead(BATTERY_PIN);

  _battery_voltage = (raw / VOLTMETER_RESOLUTION) * MAX_VOLTAGE;

  if (_battery_voltage > EPSILON && _battery_voltage < MIN_VOLTAGE) {
    stop();
    
    beep(4);

    if (malfunction_emitter.count() > 0) {
      StaticJsonDocument<64> doc;
      char buffer[64];
      
      doc["voltage"] = _battery_voltage;
  
      serializeJson(doc, buffer);

      malfunction_emitter.send(buffer, "battery-undervoltage");
    }
  } else if (sensor_emitter.count() > 0) {
    StaticJsonDocument<64> doc;
    char buffer[64];
      
    doc["voltage"] = _battery_voltage;
  
    serializeJson(doc, buffer);

    sensor_emitter.send(buffer, "battery-voltage-updated");
  }
}

/**
 * Beep the beeper.
 */
void beep(uint8_t count) {
  digitalWrite(BEEPER_PIN, HIGH);

  count--;

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
 * Calculate the final angle of a rotation from a starting point with the addition of delta
 * in radians.
 */
float calculateTargetAngle(float start, float delta) {
  float end = start + delta;
  
  if (end > M_PI) {
    end -= TWO_PI;
  } else if (end < -M_PI) {
    end += TWO_PI;
  }

  return end;
}

/**
 * Calculate the difference between a target and the current yaw.
 */
float calculateYawDelta(float current, float target) {
  float delta = target - current;

  if (fabs(delta) > M_PI) {
    float adjusted_yaw;
    
    if (current > 0.0) {
      adjusted_yaw = current - TWO_PI;
    } else {
      adjusted_yaw = current + TWO_PI;
    }

    delta = target - adjusted_yaw;
  }

  return delta;
}

/**
 * Samples a random weighted index.
 */
uint8_t randomWeightedIndex(uint16_t weights[], uint8_t n) {
  uint8_t index;
  
  long sigma = 0;

  for (index = 0; index < n; index++) {
    sigma += weights[index];
  }
 
  long delta = random(sigma);
  
  for (index = 0; index < n; index++) {
    delta -= weights[index];

    if (delta <= 0) {
      return index;
    }
  }

  return n - 1;
}

/**
 * What to do when we don't know what to do.
 */
void panicNow() {
  beep(4);

  delay(4 * (BEEP_DURATION + BEEP_DELAY));

  ESP.deepSleep(0);
}

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <VL53L1X.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <TinyGPS++.h>
#include <Ticker.h>
#include <Wire.h>
#include <FS.h>

#define SERIAL_BAUD 9600

#define AP_SSID "TankRobot"
#define AP_PASSWORD "KeepSummerSafe"
#define AP_CHANNEL 11
#define AP_MAX_CONNECTIONS 2
#define AP_HIDDEN 0

#define HTTP_PORT 80
#define HTTP_OK 200
#define HTTP_NOT_FOUND 404
#define HTTP_UNPROCESSABLE_ENTITY 422

#define BATTERY_PIN A0
#define BATTERY_SAMPLE_RATE 3000
#define MIN_VOLTAGE 6.0
#define MAX_VOLTAGE 8.4

#define FORWARD 1
#define REVERSE 0
#define LEFT 2
#define RIGHT 3

#define MOTOR_A_DIRECTION_PIN 0
#define MOTOR_A_THROTTLE_PIN 5
#define MOTOR_B_DIRECTION_PIN 2
#define MOTOR_B_THROTTLE_PIN 4
#define MIN_THROTTLE 500
#define MAX_THROTTLE 1000

#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 14
#define I2C_CLOCK 400000

#define MPU_ADDRESS 0x68
#define MPU_INTERRUPT_PIN 10
#define GYRO_CALIBRATION_LOOPS 8
#define ACCEL_CALIBRATION_LOOPS 8
#define LOW_PASS_FILTER_MODE 3
#define TEMPERATURE_SAMPLE_RATE 3000
#define TEMPERATURE_SENSITIVITY 340.0
#define TEMPERATURE_CONSTANT 36.53
#define ROTATOR_SAMPLE_RATE 50
#define ROTATOR_P_COEFF 0.33
#define ROTATOR_I_COEFF 0.33
#define ROTATOR_D_COEFF 0.33
#define ROTATOR_THRESHOLD 0.03
#define STABILIZER_SAMPLE_RATE 50
#define ROLLOVER_SAMPLE_RATE 1000
#define ROLLOVER_THRESHOLD HALF_PI

#define LIDAR_ADDRESS 0x52
#define LIDAR_SENSOR_OFFSET 10
#define LIDAR_TIMING_BUDGET 100000
#define LIDAR_SAMPLE_RATE 100
#define LIDAR_TIMEOUT 500
#define SCAN_SAMPLE_RATE 100
#define SCAN_ANGLE TWO_PI - HALF_PI
#define NUM_SCANS 4
#define COLLISION_SAMPLE_RATE LIDAR_SAMPLE_RATE
#define COLLISION_THRESHOLD 300

#define STATUS_SAMPLE_RATE 3000
#define POSITION_SAMPLE_RATE 3000

AsyncWebServer server(HTTP_PORT);

AsyncEventSource events("/events");

float _battery_voltage;

Ticker battery_timer;

int _direction = FORWARD;
unsigned int _throttle = 750;
bool _stopped = true;

MPU6050 mpu(MPU_ADDRESS);

Quaternion _q;
VectorFloat _gravity;
float _orientation[3], _yaw_lock, _temperature;
float _rotator_previous_delta, _rotator_delta_integral;
volatile bool _dmp_ready = false;
uint8_t _dmp_fifo_buffer[64];
uint16_t _dmp_packet_size;

void ICACHE_RAM_ATTR dmpDataReady();

Ticker rotator_timer, stabilizer_timer, rollover_timer;
Ticker temperature_timer;

VL53L1X lidar;

float _scan_distances[NUM_SCANS];
unsigned int _scan_count;

Ticker scan_timer, collision_timer;

TinyGPSPlus gps;

Ticker status_timer, position_timer;

/**
 * Bootstrap routine to setup the robot.
 */
void setup() {  
  setupSerial();
  
  setupSPIFFS();

  setupAccessPoint();
  setupHttpServer();

  setupBattery();
  
  setupMotors();

  setupI2C();
  setupMPU();
  setupLidar();

  initRandom();

  setupGPS();

  setupBroadcasting();
}

/**
 * Set up the serial interface.
 */
void setupSerial() {
  Serial.begin(SERIAL_BAUD);
}

/**
 * Set up the SPI Flash Filesystem.
 */
void setupSPIFFS() {
  if (SPIFFS.begin()) {
    Serial.println("SPIFFS mounted successfully");
  } else {
    Serial.println("Failed to mount SPIFFS");
  }
}

/**
 * Set up the wireless access point.
 */
void setupAccessPoint() {
  const IPAddress ip(192,168,4,1);
  const IPAddress gateway(192,168,4,1);
  const IPAddress subnet(255,255,255,0);
  
  WiFi.softAPConfig(ip, gateway, subnet);
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
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.serveStatic("/settings", SPIFFS, "/index.html");
  
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/movement/forward", HTTP_PUT, handleForward);
  server.on("/api/movement/left", HTTP_PUT, handleLeft);
  server.on("/api/movement/right", HTTP_PUT, handleRight);
  server.on("/api/movement/reverse", HTTP_PUT, handleReverse);
  server.on("/api/movement/stop", HTTP_PUT, handleStop);

  server.addHandler(new AsyncCallbackJsonWebHandler("/api/throttle", handleSetThrottle));
  server.addHandler(&events);

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

  stop();

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
  }

  Serial.print("Calibrating ");

  mpu.CalibrateGyro(GYRO_CALIBRATION_LOOPS);
  mpu.CalibrateAccel(ACCEL_CALIBRATION_LOOPS);
  
  mpu.PrintActiveOffsets();

  mpu.setDLPFMode(LOW_PASS_FILTER_MODE);
  mpu.setSleepEnabled(false);

  mpu.dmpInitialize();
  
  mpu.setDMPEnabled(true);

  _dmp_packet_size = mpu.dmpGetFIFOPacketSize();

  pinMode(MPU_INTERRUPT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT_PIN), dmpDataReady, RISING);

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
  }

  lidar.setDistanceMode(VL53L1X::Long);
  lidar.setMeasurementTimingBudget(LIDAR_TIMING_BUDGET);
  lidar.startContinuous(LIDAR_SAMPLE_RATE);
}

/**
 * Initialize the random number generator.
 */
void initRandom() {
  randomSeed(analogRead(BATTERY_PIN));

  Serial.println("Initialized random number generator");
}

/**
 * Set up the GPS module.
 */
void setupGPS() {
  Serial.println("GPS initialized");

  delay(500);

  Serial.swap();
}

/**
 * Set up event broacasting.
 */
void setupBroadcasting() {
  status_timer.attach_ms(STATUS_SAMPLE_RATE, broadcastStatus);
  
  position_timer.attach_ms(POSITION_SAMPLE_RATE, broadcastPosition);
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

    if (fifo_count % _dmp_packet_size != 0) {
      mpu.resetFIFO();
    } else if (mpu_int_status & 0x10 || fifo_count >= 1024) {
      mpu.resetFIFO();
    } else if (mpu_int_status & 0x02) {
      readDMPBuffer(fifo_count);

      updateOrientation();
    }
  }

  yield();

  while (Serial.available() > 0) {
    gps.encode(Serial.read());
  }
}

/**
 * Handle the robot status request.
 */
int handleStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<512> doc;
  char buffer[512];

  doc["throttle"] = _throttle;
  doc["battery_voltage"] = _battery_voltage;
  doc["temperature"] = _temperature;

  JsonObject position = doc.createNestedObject("position");
    
  position["lat"] = gps.location.lat();
  position["lon"] = gps.location.lng();

  doc["num_satellites"] = gps.satellites.value();
  
  serializeJson(doc, buffer);

  request->send(HTTP_OK, "application/json", buffer);
}

/**
 * Handle the forward direction request.
 */
void handleForward(AsyncWebServerRequest *request) {
  forward();

  request->send(HTTP_OK);
}

/**
 * Handle the left direction request.
 */
void handleLeft(AsyncWebServerRequest *request) {
  left();

  request->send(HTTP_OK);
}

/**
 * Handle the right direction request.
 */
void handleRight(AsyncWebServerRequest *request) {
  right();

  request->send(HTTP_OK);
}

/**
 * Handle the reverse direction request.
 * 
 * @param AsyncWebServerRequest* request
 */
void handleReverse(AsyncWebServerRequest *request) {
  reverse();

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
 * Handle the set throttle request.
 */
void handleSetThrottle(AsyncWebServerRequest *request, JsonVariant &json) {
  const JsonObject& doc = json.as<JsonObject>();

  if (!doc.containsKey("throttle")) {
    request->send(HTTP_UNPROCESSABLE_ENTITY);

    return;
  }

  setThrottle((int) doc["throttle"]);

  request->send(HTTP_OK);
}

/**
 * Catch all web route to return a 404 response if a route not found.
 */
int handleNotFound(AsyncWebServerRequest *request) {
  request->send(HTTP_NOT_FOUND);
}

/**
 * Move forward.
 */
void forward() {
  _direction = FORWARD;
  _yaw_lock = _orientation[0];

  stabilizer_timer.attach_ms(STABILIZER_SAMPLE_RATE, stabilize);
  collision_timer.attach_ms(COLLISION_SAMPLE_RATE, detectCollision);

  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

  go();
}

/**
 * Rotate the vehicle x radians to the left.
 */
void rotateLeft(float radians = HALF_PI) { 
  _direction = LEFT;
  _yaw_lock = calculateTargetAngle(_orientation[0], -radians);
  _stopped = false;

  rotator_timer.attach_ms(ROTATOR_SAMPLE_RATE, rotate);
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
 * Rotate the vehicle by x radians to the right.
 */
void rotateRight(float radians = HALF_PI) {
  _direction = RIGHT;
  _yaw_lock = calculateTargetAngle(_orientation[0], radians);
  _stopped = false;

  rotator_timer.attach_ms(ROTATOR_SAMPLE_RATE, rotate);
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
  _yaw_lock = _orientation[0];

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
 * Disengage the motors and any controls.
 */
void stop() {
  _stopped = true;
    
  rotator_timer.detach();
  stabilizer_timer.detach();
  collision_timer.detach();
  rollover_timer.detach();

  analogWrite(MOTOR_A_THROTTLE_PIN, 0);
  analogWrite(MOTOR_B_THROTTLE_PIN, 0);
}

/**
 * Set the throttle position of the motor controller.
 * 
 * @param int throttle
 */
void setThrottle(int throttle) {
  _throttle = map(throttle, 0, 100, MIN_THROTTLE, MAX_THROTTLE);

  if (!_stopped) {
    analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
    analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);
  }
}

/**
 * MPU interrupt callback to signal DMP data is ready in the buffer.
 */
void dmpDataReady() {
  _dmp_ready = true;
}

/**
 * Read the DMP FIFO buffer into memory.
 * 
 * @param int fifo_count
 */
void readDMPBuffer(int fifo_count) {
  while (fifo_count >= _dmp_packet_size) {
    mpu.getFIFOBytes(_dmp_fifo_buffer, _dmp_packet_size);

    fifo_count -= _dmp_packet_size;
  }

  _dmp_ready = false;
}

/**
 * Compute current yaw, pitch, and roll using the onboard Digital Motion Processeor.
 */
void updateOrientation() {
  mpu.dmpGetQuaternion(&_q, _dmp_fifo_buffer);

  _q.x *= -1.0;
  _q.z *= -1.0;
  
  mpu.dmpGetGravity(&_gravity, &_q);

  _gravity.x *= -1.0;
  _gravity.z *= -1.0;
 
  mpu.dmpGetYawPitchRoll(_orientation, &_q, &_gravity);
}

/**
 * Update the temperature.
 */
void updateTemperature() {
  long raw = mpu.getTemperature();
  
  _temperature = raw / TEMPERATURE_SENSITIVITY + TEMPERATURE_CONSTANT;
}

/**
 * Routine to maintain an orientation lock during rotation by actuating the motors.
 */
void rotate() {
  float delta = calculateYawDelta();

  _rotator_delta_integral += delta;
      
  float derivative = delta - _rotator_previous_delta;

  float theta = ROTATOR_P_COEFF * delta
    + ROTATOR_I_COEFF * _rotator_delta_integral
    + ROTATOR_D_COEFF * derivative;

  float alpha = min(fabs(theta) / M_PI, 1.0);

  unsigned int rotator_throttle = (int) round(alpha * _throttle);
  
  analogWrite(MOTOR_A_THROTTLE_PIN, rotator_throttle);
  analogWrite(MOTOR_B_THROTTLE_PIN, rotator_throttle);

  if (theta > 0.0) {
    if (_direction != LEFT) {      
      digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
      digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

      _direction = LEFT;
    }
  } else {
    if (_direction != RIGHT) {
      digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
      digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);

      _direction = RIGHT;
    }
  }

  if (fabs(delta) < ROTATOR_THRESHOLD) {
    _rotator_delta_integral = 0.0;
    _rotator_previous_delta = 0.0;
    
    stop();
  } else {
    _rotator_previous_delta = delta;
  }
}

/**
 * Maintain an orientation lock during linear movement by applying backoff to the side that is ahead.
 */
void stabilize() {
  float delta = calculateYawDelta();

  float beta = min(fabs(delta) / HALF_PI, 1.0);

  unsigned int backoff_throttle = (int) round((1.0 - beta) * _throttle);

  if (delta > 0.0) {
    switch (_direction) {
      case FORWARD:
        analogWrite(MOTOR_A_THROTTLE_PIN, backoff_throttle);
        
        break;

      case REVERSE:
        analogWrite(MOTOR_B_THROTTLE_PIN, backoff_throttle);
        
        break;
    }
  } else {
    switch (_direction) {
      case FORWARD:
        analogWrite(MOTOR_B_THROTTLE_PIN, backoff_throttle);
        
        break;

      case REVERSE:
        analogWrite(MOTOR_A_THROTTLE_PIN, backoff_throttle);
        
        break;
    }
  }
}

/**
 * Detect if collision with an object is iminent and stop the vehicle.
 */
void detectCollision() {
  if (lidar.ranging_data.range_mm - LIDAR_SENSOR_OFFSET < COLLISION_THRESHOLD) {
    stop();
  }
}

/**
 * Detect if the vehicle has rolled over.
 */
void detectRollover() {
  if (fabs(_orientation[1]) > ROLLOVER_THRESHOLD || fabs(_orientation[2]) > ROLLOVER_THRESHOLD) {
    stop();

    events.send("", "rollover-detected");
  }
}

/**
 * Update the battery voltage.
 */
void updateBattery() {
  unsigned int raw = analogRead(BATTERY_PIN);

  _battery_voltage = (raw / 1024.0) * MAX_VOLTAGE;
}

/**
 * Broadcast the robot status.
 */
void broadcastStatus() {
  StaticJsonDocument<128> doc;
  char buffer[128];

  doc["battery_voltage"] = _battery_voltage;
  doc["temperature"] = _temperature;
  doc["num_satellites"] = gps.satellites.value();

  serializeJson(doc, buffer);

  events.send(buffer, "status-update");
}

/**
 * Broadcast the robot's position.
 */
void broadcastPosition() {
  StaticJsonDocument<128> doc;
  char buffer[128];
    
  doc["lat"] = gps.location.lat();
  doc["lon"] = gps.location.lng();

  serializeJson(doc, buffer);

  events.send(buffer, "position-update");
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
 * Calculate the difference between the yaw lock and the current yaw.
 */
float calculateYawDelta() {
  float delta = _yaw_lock - _orientation[0];

  if (fabs(delta) > M_PI) {
    float adjusted_yaw;
    
    if (_orientation[0] > 0.0) {
      adjusted_yaw = _orientation[0] - TWO_PI;
    } else {
      adjusted_yaw = _orientation[0] + TWO_PI;
    }

    delta = _yaw_lock - adjusted_yaw;
  }

  return delta;
}

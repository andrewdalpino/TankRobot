#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <VL53L1X.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Ticker.h>
#include <Wire.h>
#include <FS.h>

#define SERIAL_BAUD 9600

#define CONFIG_FILE "/config.json"

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
#define TEMPERATURE_SAMPLE_RATE 1000
#define TEMPERATURE_SENSITIVITY 340.0
#define TEMPERATURE_CONSTANT 36.53
#define STABILIZER_SAMPLE_RATE 50
#define STABILIZER_TOLERANCE 1.0 * DEGREES_TO_RADIANS
#define ROLLOVER_SAMPLE_RATE 1000
#define ROLLOVER_THRESHOLD M_PI / 2.0

#define LIDAR_ADDRESS 0x52
#define LIDAR_SENSOR_OFFSET 50
#define LIDAR_TIMING_BUDGET 50000
#define LIDAR_SAMPLE_RATE 50
#define LIDAR_TIMEOUT 500
#define COLLISION_SAMPLE_RATE LIDAR_SAMPLE_RATE
#define COLLISION_THRESHOLD_MM 300

#define GPS_RX_PIN 13
#define GPS_TX_PIN 15
#define GPS_BAUD 9600

#define DEGREES_TO_RADIANS M_PI / 180.0
#define RADIANS_TO_DEGREES 180.0 / M_PI

StaticJsonDocument<512> _config;

AsyncWebServer server(HTTP_PORT);

AsyncEventSource events("/events");

float _battery_voltage;
Ticker battery_timer;

unsigned int _direction = FORWARD;
unsigned int _throttle = 750;
bool _stopped = true;

MPU6050 mpu(MPU_ADDRESS);

Quaternion _q;
VectorFloat _gravity;
float _orientation[3], _yaw_lock, _temperature;
uint8_t _dmp_fifo_buffer[64];
uint16_t _dmp_packet_size;
volatile bool _dmp_ready = false;

Ticker stabilizer_timer;
Ticker rollover_timer, temperature_timer;

void ICACHE_RAM_ATTR dmpDataReady();

VL53L1X lidar;

Ticker collision_timer;

SoftwareSerial ss(GPS_RX_PIN, GPS_TX_PIN);

TinyGPSPlus gps;

/**
 * Bootstrap routine to setup the robot.
 */
void setup() {  
  setupSerial();
  
  setupSPIFFS();
  loadConfig();

  setupAccessPoint();
  setupHttpServer();

  setupBattery();
  
  setupMotors();

  setupI2C();
  setupMPU();
  setupLidar();

  setupGPS();

  initRandom();
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
 * Load the configuration file into memory.
 */
void loadConfig() {
  char buffer[512];
    
  File f = SPIFFS.open(CONFIG_FILE, "r");

  f.readBytes(buffer, f.size());
  f.close();

  deserializeJson(_config, buffer);
}

/**
 * Set up the wireless access point.
 */
void setupAccessPoint() {
  const char* ssid = _config["ap"]["ssid"];
  const char* password = _config["ap"]["password"];
  int channel = _config["ap"]["channel"];
  bool hidden = _config["ap"]["hidden"];
  int max_connections = _config["ap"]["max_connections"];
  const IPAddress ip(192,168,4,1);
  const IPAddress gateway(192,168,4,1);
  const IPAddress subnet(255,255,255,0);
  
  WiFi.softAPConfig(ip, gateway, subnet);
  WiFi.softAP(ssid, password, channel, hidden, max_connections);

  Serial.println("Access point ready");
  Serial.print("SSID: ");
  Serial.println(ssid);
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
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
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

void setupGPS() {
  ss.begin(GPS_BAUD);

  Serial.println("GPS initialized");
}

/**
 * Initialize the random number generator.
 */
void initRandom() {
  randomSeed(analogRead(BATTERY_PIN));

  Serial.println("Initialized random number generator");
}

/**
 * Main event loop.
 */
void loop() {    
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

  if (lidar.dataReady()) {
    lidar.read(false);
  }

  while (ss.available() > 0) {
    gps.encode(ss.read());
  }
}

/**
 * Handle the robot status request.
 * 
 * @param AsyncWebServerRequest* request
 */
int handleStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<512> doc;
  char buffer[512];

  doc["direction"] = _direction;
  doc["throttle"] = _throttle;
  doc["stopped"] = _stopped;
  doc["battery_voltage"] = _battery_voltage;
  
  serializeJson(doc, buffer);

  request->send(HTTP_OK, "application/json", buffer);
}

/**
 * Handle the forward direction request.
 * 
 * @param AsyncWebServerRequest* request
 */
void handleForward(AsyncWebServerRequest *request) {
  forward();

  request->send(HTTP_OK);
}

/**
 * Handle the left direction request.
 * 
 * @param AsyncWebServerRequest* request
 */
void handleLeft(AsyncWebServerRequest *request) {
  left();

  request->send(HTTP_OK);
}

/**
 * Handle the right direction request.
 * 
 * @param AsyncWebServerRequest* request
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
 * 
 * @param AsyncWebServerRequest* request
 * @param JsonVariant &json
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
 * 
 * @param AsyncWebServerRequest* request
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

  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

  stabilizer_timer.attach_ms(STABILIZER_SAMPLE_RATE, stabilize);
  collision_timer.attach_ms(COLLISION_SAMPLE_RATE, detectCollision);

  go();
}

/**
 * Rotate the vehicle x radians to the left.
 * 
 * @param float radians
 */
void rotateLeft(float radians) {
  _direction = LEFT;   
  _yaw_lock = calculateAngle(_orientation[0], -radians);

  stabilizer_timer.attach_ms(STABILIZER_SAMPLE_RATE, stabilize);

  go();
}

/**
 * Turn in the left direction.
 */
void left() {
  _direction = LEFT;

  digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

  go();
}

/**
 * Rotate the vehicle by x radians to the right.
 * 
 * @param float radians
 */
void rotateRight(float radians) {
  _direction = RIGHT;
  _yaw_lock = calculateAngle(_orientation[0], radians);

  stabilizer_timer.attach_ms(STABILIZER_SAMPLE_RATE, stabilize);

  go();
}

/**
 * Turn in the right direction.
 */
void right() {
  _direction = RIGHT;

  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);

  go();
}

/**
 * Move in reverse.
 */
void reverse() {
  _direction = REVERSE;
  _yaw_lock = _orientation[0];

  digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
  digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);

  stabilizer_timer.attach_ms(STABILIZER_SAMPLE_RATE, stabilize);

  go();
}

/**
 * Engage the motors.
 */
void go() {
  _stopped = false;

  analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
  analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);

  rollover_timer.attach_ms(ROLLOVER_SAMPLE_RATE, detectRollover);
}

/**
 * Disengage the motors and any controls in process.
 */
void stop() {
  analogWrite(MOTOR_A_THROTTLE_PIN, 0);
  analogWrite(MOTOR_B_THROTTLE_PIN, 0);
  
  stabilizer_timer.detach();
  collision_timer.detach();
  rollover_timer.detach();

  _stopped = true;
}

/**
 * Set the throttle position of the motor controller.
 * 
 * @param int throttle
 */
void setThrottle(int throttle) {
  _throttle = max(MIN_THROTTLE, min(throttle, MAX_THROTTLE));

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
  
  mpu.dmpGetGravity(&_gravity, &_q);
  
  mpu.dmpGetYawPitchRoll(_orientation, &_q, &_gravity);
}

/**
 * Update the temperature.
 */
void updateTemperature() {
  long raw = mpu.getTemperature();
  
  _temperature = raw / TEMPERATURE_SENSITIVITY + TEMPERATURE_CONSTANT;

  if (events.count() > 0) {
    StaticJsonDocument<128> doc;
    char buffer[128];
  
    doc["temperature"] = _temperature;

    serializeJson(doc, buffer);

    events.send(buffer, "temperature-update");
  }
}

/**
 * Update the battery voltage and broadcast the event.
 */
void updateBattery() {
  unsigned int raw = analogRead(BATTERY_PIN);

  _battery_voltage = raw / 1024.0 * MAX_VOLTAGE;

  if (events.count() > 0) {
    StaticJsonDocument<128> doc;
    char buffer[128];
  
    doc["battery_voltage"] = _battery_voltage;

    serializeJson(doc, buffer);

    events.send(buffer, "battery-update");
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
 * Detect if collision with an object is iminent and stop the vehicle.
 */
void detectCollision() {
  if (lidar.ranging_data.range_mm < COLLISION_THRESHOLD_MM) {
    stop();
  }
}

/**
 * Calculate the final angle of a rotation from a starting point with the addition of delta
 * in radians.
 * 
 * @param float start
 * @param float delta
 * @return float
 */
float calculateAngle(float start, float delta) {
  float end = start + delta;
  
  if (fabs(end) > M_PI) {
    end = -(M_PI - fmod(end, M_PI));
  }

  return end;
}

/**
 * Check that the vehicle orientation is within a specified tolerance of the yaw lock and
 * momentarily let off the throttle on the side that is ahead.
 */
void stabilize() {
  float delta = _yaw_lock - _orientation[0];
  
  // ...
}

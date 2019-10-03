#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Ticker.h>
#include <Wire.h>
#include <FS.h>

#define SERIAL_BAUD 9600

#define HTTP_PORT 80
#define HTTP_OK 200
#define HTTP_NOT_FOUND 404
#define HTTP_UNPROCESSABLE_ENTITY 422

#define WEBSOCKETS_PORT 9090

#define BATTERY_PIN A0
#define BATTERY_SAMPLE_RATE 3000
#define MIN_VOLTAGE 6.0
#define MAX_VOLTAGE 8.4

#define MOTOR_A_DIRECTION_PIN 0
#define MOTOR_A_THROTTLE_PIN 5
#define MOTOR_B_DIRECTION_PIN 2
#define MOTOR_B_THROTTLE_PIN 4
#define FORWARD 1
#define REVERSE 0
#define MIN_THROTTLE 500
#define MAX_THROTTLE 1000

#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 14
#define I2C_CLOCK 400000

#define MPU_ADDRESS 0x68
#define INTERRUPT_PIN 13
#define CALIBRATION_LOOPS 8
#define TEMPERATURE_SENSITIVITY 340.0
#define TEMPERATURE_CONSTANT 36.53
#define TEMPERATURE_SAMPLE_RATE 5000
#define ROTATE_SAMPLE_RATE 10
#define ROTATION_TOLERANCE 1.0 * DEGREES_TO_RADIANS
#define STABILIZER_SAMPLE_RATE 50
#define STABILIZER_THRESHOLD 2.0 * DEGREES_TO_RADIANS
#define STABILIZER_TOLERANCE 0.05 * STABILIZER_THRESHOLD
#define BACKOFF_AMOUNT 0.3
#define ROLLOVER_SAMPLE_RATE 1000
#define ROLLOVER_THRESHOLD M_PI / 2.0

#define SONAR_TRIGGER_PIN 10
#define SONAR_ECHO_PIN 15
#define SONAR_OFFSET 3
#define MAX_PING_RANGE 500
#define PING_RANGE 250
#define PING_TIMEOUT PING_RANGE * CM_ROUNDTRIP_US
#define MAX_PING_TIMEOUT MAX_PING_RANGE * CM_ROUNDTRIP_US
#define ECHO_OFFSET SONAR_OFFSET * CM_ROUNDTRIP_US
#define NUM_PINGS 3
#define SONAR_SAMPLE_RATE 50

#define DEGREES_TO_RADIANS M_PI / 180.0

#define SPEED_SOUND 331.5
#define SOUND_TEMP_COEFF 0.60
#define CM_ROUNDTRIP_US 57

StaticJsonDocument<500> config;

ESP8266WebServer server(HTTP_PORT);

WebSocketsServer socket = WebSocketsServer(WEBSOCKETS_PORT);

Ticker battery_timer;
float _battery_voltage;

char* _direction;
unsigned int _throttle = MAX_THROTTLE - 0.5 * (MAX_THROTTLE - MIN_THROTTLE);
bool _stopped = true;

MPU6050 mpu(MPU_ADDRESS);

Ticker temperature_timer, rotation_timer, stabilizer_timer, rollover_timer;

Quaternion _q;
VectorFloat _gravity;
float _orientation[3];
float _temperature;
uint8_t _dmp_fifo_buffer[64];
uint16_t _dmp_packet_size;
bool _dmp_ready = false;
bool _stabilize = true;
bool _backing_off = false;
bool _rotating = false;
float _yaw_lock;

void ICACHE_RAM_ATTR dmpDataReady();

Ticker sonar_timer;
float _adjusted_speed_sound_us = SPEED_SOUND / 10000.0;
unsigned int _ping_buffer[NUM_PINGS];
uint8_t _ping_counter;
float _distance_to_object;

/**
 * Bootstrap routine to setup the robot.
 */
void setup() {  
  setupSerial();
  setupSPIFFS();
  loadConfig();

  setupAccessPoint();
  setupHttpServer();
  setupWebsocketsServer();

  setupBattery();
  
  setupMotors();

  setupI2C();
  setupMPU();
  setupDMP();

  setupSonar();
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
  char buffer[500];
    
  File f = SPIFFS.open("/config.json", "r");

  f.readBytes(buffer, f.size());

  deserializeJson(config, buffer);
}

/**
 * Set up the wireless access point.
 */
void setupAccessPoint() {
  const char* ssid = config["ap"]["ssid"];
  const char* password = config["ap"]["password"];
  int channel = config["ap"]["channel"];
  bool hidden = config["ap"]["hidden"];
  int max_connections = config["ap"]["max_connections"];
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
  server.serveStatic("/", SPIFFS, "/index.html");
  server.serveStatic("/app.js", SPIFFS, "/app.js");
  server.serveStatic("/app.css", SPIFFS, "/app.css");
  server.serveStatic("/fa-solid-900.woff", SPIFFS, "/fa-solid-900.woff");
  server.serveStatic("/fa-solid-900.woff2", SPIFFS, "/fa-solid-900.woff2");
  server.serveStatic("/app-icon-small.png", SPIFFS, "/app-icon-small.png");
  server.serveStatic("/app-icon-large.png", SPIFFS, "/app-icon-large.png");
  server.serveStatic("/plucky.ogg", SPIFFS, "/plucky.ogg");
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/movement/forward", HTTP_PUT, handleForward);
  server.on("/api/movement/reverse", HTTP_PUT, handleReverse);
  server.on("/api/movement/left", HTTP_PUT, handleLeft);
  server.on("/api/movement/right", HTTP_PUT, handleRight);
  server.on("/api/movement/stop", HTTP_PUT, handleStop);
  server.on("/api/movement/stabilizer", HTTP_PUT, handleEnableStabilizer);
  server.on("/api/movement/stabilizer", HTTP_DELETE, handleDisableStabilizer);
  server.on("/api/throttle", HTTP_PUT, handleSetThrottle);
  server.onNotFound(handleNotFound);

  server.begin();

  Serial.print("HTTP server listening on port ");
  Serial.println(HTTP_PORT);
}

/**
 * Set up the websockets server.
 */
void setupWebsocketsServer() {
  socket.begin();
  socket.onEvent(handleWebSocket);

  Serial.print("Websockets server listening on port ");
  Serial.println(WEBSOCKETS_PORT);
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
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU initialized");
  } else {
    Serial.println("MPU failure");
  }

  Serial.print("Calibrating ");

  mpu.CalibrateAccel(CALIBRATION_LOOPS);
  mpu.CalibrateGyro(CALIBRATION_LOOPS);
  mpu.PrintActiveOffsets();

  mpu.setSleepEnabled(false);
  mpu.setStandbyXAccelEnabled(true);
  mpu.setStandbyYAccelEnabled(true);
  mpu.setStandbyZAccelEnabled(true);

  temperature_timer.attach_ms(TEMPERATURE_SAMPLE_RATE, updateTemperature);
}

/**
 * Set up the Digital Motion Processor.
 */
void setupDMP() {
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);

  _dmp_packet_size = mpu.dmpGetFIFOPacketSize();

  pinMode(INTERRUPT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

  Serial.println("DMP initialized");
}

/**
 * Set up the sonar distance sensor.
 */
void setupSonar() {
  pinMode(SONAR_TRIGGER_PIN, OUTPUT);
  pinMode(SONAR_ECHO_PIN, INPUT);

  sonar_timer.attach_ms(SONAR_SAMPLE_RATE, updateDistanceToObject);

  Serial.println("Sonar enabled");
}

/**
 * Main event loop.
 */
void loop() {
  server.handleClient();
  
  socket.loop();
  
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
}

/**
 * Handle the robot status request.
 */
void handleStatus() {
  StaticJsonDocument<500> doc;
  char buffer[500];

  doc["direction"] = _direction;
  doc["throttle"] = _throttle;
  doc["stopped"] = _stopped;
  doc["stabilize"] = _stabilize;
  doc["battery_voltage"] = _battery_voltage;
  doc["temperature"] = _temperature;
  
  serializeJson(doc, buffer);

  server.send(HTTP_OK, "application/json", buffer);
}

/**
 * Handle the forward direction request.
 */
void handleForward() {
  forward();

  server.send(HTTP_OK);
}

/**
 * Handle the reverse direction request.
 */
void handleReverse() {
  reverse();

  server.send(HTTP_OK);
}

/**
 * Handle the left direction request.
 */
void handleLeft() {
  left();

  server.send(HTTP_OK);
}

/**
 * Handle the right direction request.
 */
void handleRight() {
  right();

  server.send(HTTP_OK);
}

/**
 * Handle the stop movement request.
 */
void handleStop() {
  stop();

  server.send(HTTP_OK);
}

/**
 * Handle the set throttle request.
 */
void handleSetThrottle() {
  StaticJsonDocument<100> doc;

  deserializeJson(doc, server.arg("plain"));

  if (!doc.containsKey("throttle")) {
    server.send(HTTP_UNPROCESSABLE_ENTITY);

    return;
  }

  setThrottle(doc["throttle"]);

  server.send(HTTP_OK);
}

/**
 * Handle an enable stabilizer request.
 */
void handleEnableStabilizer() {
  setStabilizer(true);

  server.send(HTTP_OK);
}

/**
 * Handle a disable stabilizer request.
 */
void handleDisableStabilizer() {
  setStabilizer(false);

  server.send(HTTP_OK);
}

/**
 * Catch all web route to return a 404 response if a route not found.
 */
int handleNotFound() {
  server.send(HTTP_NOT_FOUND, "text/html", "<h1>Page not found</h1>");
}

/**
 * Move forward.
 */
void forward() {
  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

  _direction = "forward";
  _yaw_lock = _orientation[0];
  
  if (_stabilize) {
    stabilizer_timer.attach_ms(STABILIZER_SAMPLE_RATE, stabilize);
  }

  go();
}

/**
 * Turn 90 degrees to the left.
 */
void turnLeft() {
  rotateLeft(90 * DEGREES_TO_RADIANS);
}

/**
 * Rotate the vehicle x radians to the left.
 * 
 * @param float radians
 */
void rotateLeft(float radians) {
  left();
    
  _yaw_lock = calculateAngle(_orientation[0], -radians);

  rotation_timer.attach_ms(ROTATE_SAMPLE_RATE, rotate);
}

/**
 * Turn left.
 */
void left() {
  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);

  _direction = "left";

  go();
}

/**
 * Turn 90 degrees to the right.
 */
void turnRight() {
  rotateRight(90 * DEGREES_TO_RADIANS);
}

/**
 * Rotate the vehicle by x radians to the right.
 * 
 * @param float radians
 */
void rotateRight(float radians) {
  right();

  _yaw_lock = calculateAngle(_orientation[0], radians);

  rotation_timer.attach_ms(ROTATE_SAMPLE_RATE, rotate);
}

/**
 * Turn right.
 */
void right() {
  digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

  _direction = "right";

  go();
}

/**
 * Move in reverse.
 */
void reverse() {
  digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
  digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);

  _direction = "reverse";
  _yaw_lock = _orientation[0];

  if (_stabilize) {
    stabilizer_timer.attach_ms(STABILIZER_SAMPLE_RATE, stabilize);
  }

  go();
}

/**
 * Engage the motors.
 */
void go() {
  analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
  analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);

  rollover_timer.attach_ms(ROLLOVER_SAMPLE_RATE, detectRollover);
  rotation_timer.detach();

  _stopped = false;
}

/**
 * Disengage the motors.
 */
void stop() {
  analogWrite(MOTOR_A_THROTTLE_PIN, 0);
  analogWrite(MOTOR_B_THROTTLE_PIN, 0);

  rotation_timer.detach();
  stabilizer_timer.detach();
  rollover_timer.detach();

  _stopped = true;
  _direction = "";
}

/**
 * Actuate the throttle of the motor controller.
 * 
 * @param int speed
 */
void setThrottle(int throttle) {
  _throttle = max(MIN_THROTTLE, min(throttle, MAX_THROTTLE));

  if (!_stopped) {
    analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
    analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);
  }
}

/**
 * Turn the linear stabilizer on and off.
 * 
 * @param bool stabilize
 */
void setStabilizer(bool stabilize) {
  _stabilize = stabilize;
}

/**
 * MPU interrupt callback to signal DMP data is ready.
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
 * Compute current yaw, pitch, and roll using the onboard DMP.
 */
void updateOrientation() {
  mpu.dmpGetQuaternion(&_q, _dmp_fifo_buffer);
  mpu.dmpGetGravity(&_gravity, &_q);
  mpu.dmpGetYawPitchRoll(_orientation, &_q, &_gravity);
}

/**
 * Update the temperature reading from the MPU and broadcast to connected sockets.
 */
void updateTemperature() {
  int32_t raw = mpu.getTemperature();
  
  _temperature = raw / TEMPERATURE_SENSITIVITY + TEMPERATURE_CONSTANT;

  _adjusted_speed_sound_us = (SPEED_SOUND + SOUND_TEMP_COEFF * _temperature) / 10000.0;

  if (socket.connectedClients() > 0) {
    StaticJsonDocument<200> doc;
    char buffer[200];
  
    doc["name"] = "temperature-update";
    doc["temperature"] = _temperature;

    serializeJson(doc, buffer);

    socket.broadcastTXT(buffer);
  }
}

/**
 * Update the battery voltage and broadcast the event to any connected websockets clients.
 */
void updateBattery() {
  uint8_t raw = analogRead(BATTERY_PIN);

  _battery_voltage = raw / 1024.0 * MAX_VOLTAGE;

  if (socket.connectedClients() > 0) {
    StaticJsonDocument<200> doc;
    char buffer[200];
  
    doc["name"] = "battery-update";
    doc["battery_voltage"] = _battery_voltage;

    serializeJson(doc, buffer);

    socket.broadcastTXT(buffer);
  }
}

/**
 * Detect if the vehicle has rolled over.
 */
void detectRollover() {
  if (fabs(_orientation[1]) > ROLLOVER_THRESHOLD || fabs(_orientation[2]) > ROLLOVER_THRESHOLD) {
    stop();

    if (socket.connectedClients() > 0) {
      StaticJsonDocument<100> doc;
      char buffer[100];
  
      doc["name"] = "rollover-detected";

      serializeJson(doc, buffer);

      socket.broadcastTXT(buffer);
    }
  }
}

/**
 * Check that the vehicle is within a threshold of the yaw lock and momentarily let off
 * the throttle on the side that is ahead.
 */
void stabilize() {
  float delta = _yaw_lock - _orientation[0];
  
  if (!_backing_off) {
    if (fabs(delta) > STABILIZER_THRESHOLD) {
      unsigned int backoff_throttle = round(_throttle * (1.0 - BACKOFF_AMOUNT));
  
      if (_direction == "forward") {
        if (delta > 0.0) {
          analogWrite(MOTOR_A_THROTTLE_PIN, backoff_throttle);
        } else {
          analogWrite(MOTOR_B_THROTTLE_PIN, backoff_throttle);
        }
      } else if (_direction == "reverse") {
        if (delta < 0.0) {
          analogWrite(MOTOR_A_THROTTLE_PIN, backoff_throttle);
        } else {
          analogWrite(MOTOR_B_THROTTLE_PIN, backoff_throttle);
        }
      }

      _backing_off = true;
    }
  } else if (fabs(delta) < STABILIZER_TOLERANCE) {
    if (_direction == "forward") {
      analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
      analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);
    } else if (_direction == "reverse") {
      analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
      analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);
    }

    _backing_off = false;
  }
}

/**
 * Rotate the vehicle.
 */
void rotate() {
  float delta = _yaw_lock - _orientation[0];
  
  if (fabs(delta) < ROTATION_TOLERANCE) {
    stop();

    rotation_timer.detach();
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
 * Update the distance from the robot to an object using the sonar.
 */
void updateDistanceToObject() {
  unsigned int t = ping(PING_TIMEOUT);

  _ping_buffer[_ping_counter] = t;

  _ping_counter++;
  
  if (_ping_counter >= NUM_PINGS) {
    int mu = median(_ping_buffer) - ECHO_OFFSET;

    if (mu > 0) {
      _distance_to_object = echoToDistance(mu);
    } else {
      _distance_to_object = INFINITY;
    }

    _ping_counter = 0;
  }
}

/**
 * Calculate the echo time in microseconds to distance in centimeters using the adjusted
 * speed of sound constant.
 */
float echoToDistance(int t) {
  return (t / 2.0) * _adjusted_speed_sound_us;
}

/**
 * Ping using the sonar and return the duration of the echo in microseconds.
 * 
 * @param int timeout
 * @returns unsigned int
 */
unsigned int ping(int timeout) {
  digitalWrite(SONAR_TRIGGER_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(SONAR_TRIGGER_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(SONAR_TRIGGER_PIN, LOW);

  unsigned int t = pulseIn(SONAR_ECHO_PIN, HIGH, min(timeout, MAX_PING_TIMEOUT));

  return t;
}

/**
 * Return the median value of an array of echo timestamps.
 * 
 * @param unsigned int data[]
 * @return unsigned int
 */
unsigned int median(unsigned int data[]) {
  unsigned int median;
  
  uint8_t n = sizeof(data) / sizeof(int);

  bsort(data, n);

  uint8_t mid = n / 2;

  if (n % 2 == 1) {
    median = data[mid];
  } else {
    median = (data[mid - 1] + data[mid]) / 2;
  }

  return median;
}

/**
 * Sort an array of n integer timestamps from lowest to highest using the bubble sort
 * algorithm.
 * 
 * @param unsigned int data[]
 * @param uint8_t n
 */
void bsort(unsigned int data[], uint8_t n) {
  uint8_t i, j, k;
  bool swapped;
   
  for (i = 0; i < n - 1; i++) {
    swapped = false;
     
    for (j = 0; j < n - i - 1; j++) {
      k = j + 1;
      
      if (data[j] > data[k]) {
        float temp = data[j];

        data[j] = data[k]; 
        data[k] = temp; 
           
        swapped = true;
      }
    } 
   
    if (!swapped) {
      break; 
    }
  }
}

 /**
 * Handle a websockets request.
 */
void handleWebSocket(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  //
}

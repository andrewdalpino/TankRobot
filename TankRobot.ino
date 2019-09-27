#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Ticker.h>
#include <Wire.h>
#include <FS.h>

#define SERIAL_BAUD 9600

#define AP_CHANNEL 1
#define MAX_CONNECTIONS 2
#define HIDDEN_SSID false

#define HTTP_PORT 80
#define HTTP_OK 200
#define HTTP_NOT_FOUND 404
#define HTTP_UNPROCESSABLE_ENTITY 422

#define WEBSOCKETS_PORT 9090

#define MOTOR_A_DIRECTION_PIN 0
#define MOTOR_A_THROTTLE_PIN 5
#define MOTOR_B_DIRECTION_PIN 2
#define MOTOR_B_THROTTLE_PIN 4
#define FORWARD 1
#define REVERSE 0
#define MIN_THROTTLE 500
#define MAX_THROTTLE 1000
#define DEFAULT_THROTTLE 750

#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 14
#define I2C_CLOCK 400000

#define MPU_ADDRESS 0x68
#define INTERRUPT_PIN 10
#define LSB_SENSITIVITY 8192.0
#define TEMPERATURE_SENSITIVITY 340.0
#define TEMPERATURE_CONSTANT 36.53
#define TEMPERATURE_SAMPLE_RATE 5000
#define YAW_LOCK_TOLERANCE 2.5 * M_PI / 180.0
#define BACKOFF_THROTTLE 0
#define BACKOFF_DURATION 10

#define BATTERY_PIN A0
#define BATTERY_SAMPLE_RATE 2500
#define MIN_VOLTAGE 6.0
#define MAX_VOLTAGE 8.4

#define TETHER_SAMPLE_RATE 250

const String _ssid = "Kirby The Tank Robot";
const String _password = "KeepSummerSafe";

const IPAddress ip(192,168,4,1);
const IPAddress gateway(192,168,4,1);
const IPAddress subnet(255,255,255,0);

ESP8266WebServer server(HTTP_PORT);

WebSocketsServer socket = WebSocketsServer(WEBSOCKETS_PORT);

unsigned int _throttle = DEFAULT_THROTTLE;
String _direction = "forward";
bool _stopped = true;

MPU6050 mpu(MPU_ADDRESS);

Quaternion _q;
VectorFloat _gravity;
VectorInt16 _aa, _aa_real, _aa_world;
float _orientation[3];
float _temperature;
float _yaw_lock;
uint16_t _dmp_packet_size;
uint8_t _dmp_fifo_buffer[64];
bool _dmp_ready = false;
bool _stabilize = true;

Ticker temperature_timer;

float _battery_voltage;

Ticker battery_timer;

Ticker tether_timer;

void ICACHE_RAM_ATTR dmpDataReady();

/**
 * Bootstrap routine to run once.
 */
void setup() {
  Serial.begin(SERIAL_BAUD);
  
  WiFi.softAPConfig(ip, gateway, subnet);
  WiFi.softAP(_ssid, _password, AP_CHANNEL, HIDDEN_SSID, MAX_CONNECTIONS);

  Serial.println("Access point ready");
  Serial.print("SSID: ");
  Serial.println(_ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("MAC Address: ");
  Serial.println(WiFi.softAPmacAddress());

  if (SPIFFS.begin()) {
    Serial.println("SPIFFS mounted successfully");
  } else {
    Serial.println("Failed to mount SPIFFS");
  }

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

  Serial.print("HTTP Server listening on port ");
  Serial.println(HTTP_PORT);
  
  socket.begin();
  socket.onEvent(handleWebSocket);

  Serial.print("Websockets Server listening on port ");
  Serial.println(WEBSOCKETS_PORT);

  pinMode(BATTERY_PIN, INPUT);
  
  battery_timer.attach_ms(BATTERY_SAMPLE_RATE, checkBattery);

  Serial.println("Battery voltmeter initialized");

  pinMode(MOTOR_A_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_A_THROTTLE_PIN, OUTPUT);

  pinMode(MOTOR_B_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_B_THROTTLE_PIN, OUTPUT);

  stop();

  Serial.println("Motors initialized");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK);

  pinMode(INTERRUPT_PIN, INPUT);

  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU initialized");
  } else {
    Serial.println("MPU failure");
  }

  Serial.println("Calibrating gyroscope and accelerometer");

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  mpu.setSleepEnabled(false);

  _dmp_packet_size = mpu.dmpGetFIFOPacketSize();

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

  temperature_timer.attach_ms(TEMPERATURE_SAMPLE_RATE, updateTemperature);

  Serial.println("DMP initialized");
  
  tether_timer.attach_ms(TETHER_SAMPLE_RATE, checkTether);

  Serial.println("WiFi tether activated");
}

/**
 * Main event loop.
 */
void loop() {
  if (_dmp_ready) {
    uint8_t mpu_int_status = mpu.getIntStatus();

    if (mpu_int_status & 0x02) {
      readDmpBuffer();
      
      updateOrientation();
    } else if (mpu_int_status & 0x10) {
      mpu.resetFIFO();
    }

    _dmp_ready = false;
  }

  if (!_stopped && _stabilize) {
    if (_direction == "forward" || _direction == "reverse") {
      float delta = _yaw_lock - _orientation[0];
  
      if (fabs(delta) > YAW_LOCK_TOLERANCE) {
        stabilize(delta);
      }
    }
  }

  server.handleClient();
  
  socket.loop();
}

/**
 * Handle the robot status request.
 */
void handleStatus() {
  StaticJsonDocument<300> doc;
  char buffer[300];

  doc["throttle"] = _throttle;
  doc["direction"] = _direction;
  doc["stabilize"] = _stabilize;
  doc["stopped"] = _stopped;
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
 * Handle the set speed request.
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
 * Handle an enable movement stabilizer request.
 */
void handleEnableStabilizer() {
  setStabilizer(true);

  server.send(HTTP_OK);
}

/**
 * Handle a disable movement stabilizer request.
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
 * Handle a websockets request.
 */
void handleWebSocket(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  //
}

/**
 * Move forward.
 */
void forward() {
  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

  _direction = "forward";
  _yaw_lock = _orientation[0];

  go();
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

  go();
}

/**
 * Engage the motors and move in a direction at a particular speed.
 */
void go() {
  analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
  analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);

  _stopped = false;
}

/**
 * Disengage the motors and come to a stop.
 */
void stop() {
  analogWrite(MOTOR_A_THROTTLE_PIN, 0);
  analogWrite(MOTOR_B_THROTTLE_PIN, 0);

  _stopped = true;
  _direction = "";
}

/**
 * Actuate the throttle of the motors.
 * 
 * @param int speed
 */
void setThrottle(int throttle) {
  _throttle = max(MIN_THROTTLE, min(throttle, MAX_THROTTLE));

  if (!_stopped) {
    go();
  }
}

/**
 * Turn the linear movement stabilizer on and off.
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
 * Read the MPU data buffer.
 */
void readDmpBuffer() {
  uint16_t fifo_count = mpu.getFIFOCount();
  
  while (fifo_count >= _dmp_packet_size) {
    mpu.getFIFOBytes(_dmp_fifo_buffer, _dmp_packet_size);

    fifo_count -= _dmp_packet_size;
  }
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
 * Compute the real world acceleration using the DMP.
 */
void updateAcceleration() {
  mpu.dmpGetAccel(&_aa, _dmp_fifo_buffer);
  mpu.dmpGetLinearAccel(&_aa_real, &_aa, &_gravity);
  mpu.dmpGetLinearAccelInWorld(&_aa_world, &_aa_real, &_q);
}

/**
 * Momentarily let off the throttle on the side that is ahead.
 * 
 * @param float delta
 */
void stabilize(float delta) {
  if (_direction == "forward") {
    if (delta > 0.0) {
      analogWrite(MOTOR_A_THROTTLE_PIN, BACKOFF_THROTTLE);

      delay(BACKOFF_DURATION);

      analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
    } else {
      analogWrite(MOTOR_B_THROTTLE_PIN, BACKOFF_THROTTLE);

      delay(BACKOFF_DURATION);

      analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);
    }
  } else if (_direction == "reverse") {
    if (delta < 0.0) {
      analogWrite(MOTOR_A_THROTTLE_PIN, BACKOFF_THROTTLE);

      delay(BACKOFF_DURATION);

      analogWrite(MOTOR_A_THROTTLE_PIN, _throttle);
    } else {
      analogWrite(MOTOR_B_THROTTLE_PIN, BACKOFF_THROTTLE);

      delay(BACKOFF_DURATION);

      analogWrite(MOTOR_B_THROTTLE_PIN, _throttle);
    }
  }
}

/**
 * Update the temperature reading from the MPU and broadcast to connected sockets.
 */
void updateTemperature() {  
  int raw = mpu.getTemperature();

  _temperature = raw / TEMPERATURE_SENSITIVITY + TEMPERATURE_CONSTANT;

  if (socket.connectedClients() > 0) {
    StaticJsonDocument<100> doc;
    char buffer[100];
  
    doc["name"] = "temperature-update";
    doc["temperature"] = _temperature;

    serializeJson(doc, buffer);

    socket.broadcastTXT(buffer);
  }
}

/**
 * Update the battery voltage and broadcast the event to any connected websockets clients.
 */
void checkBattery() {
  float raw = analogRead(BATTERY_PIN);

  _battery_voltage = raw / 1024.0 * MAX_VOLTAGE;

  if (_battery_voltage <= MIN_VOLTAGE) {
    stop();
  }

  if (socket.connectedClients() > 0) {
    StaticJsonDocument<100> doc;
    char buffer[100];
  
    doc["name"] = "battery-update";
    doc["battery_voltage"] = _battery_voltage;

    serializeJson(doc, buffer);

    socket.broadcastTXT(buffer);
  }
}

/**
 * Check that a client is connected or stop movement.
 */
void checkTether() {
  if (WiFi.softAPgetStationNum() == 0) {
    stop();
  }
}

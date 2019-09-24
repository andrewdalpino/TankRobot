#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Ticker.h>
#include <Wire.h>
#include <FS.h>

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
#define MIN_THROTTLE 250
#define MAX_THROTTLE 1000
#define DEFAULT_THROTTLE 625

#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 14
#define I2C_CLOCK 400000

#define MPU_ADDRESS 0x68
#define INTERRUPT_PIN 16
#define DMP_SAMPLE_RATE 50
#define LSB_SENSITIVITY 8192.0

#define BATTERY_PIN A0
#define BATTERY_SAMPLE_RATE 5000

#define TETHER_SAMPLE_RATE 250

#define SERIAL_BAUD 9600

char* _ssid = "Kirby The Tank Robot";
char* _password = "KeepSummerSafe";

const IPAddress ip(192,168,4,1);
const IPAddress gateway(192,168,4,1);
const IPAddress subnet(255,255,255,0);

ESP8266WebServer server(HTTP_PORT);

WebSocketsServer socket = WebSocketsServer(WEBSOCKETS_PORT);

unsigned int _throttle = DEFAULT_THROTTLE;
char* _direction = "forward";
bool _stopped = true;

MPU6050 mpu(MPU_ADDRESS);

Quaternion q;
VectorInt16 aa, aaReal, aaWorld;
VectorFloat gravity;
uint16_t _dmp_packet_size;
uint8_t _dmp_fifo_buffer[64];
bool _dmp_ready = false;

float _battery_level = 1.0;

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
  server.on("/api/throttle", HTTP_PUT, handleSetThrottle);
  server.onNotFound(handleNotFound);

  server.begin();

  Serial.print("HTTP Server listening on port: ");
  Serial.println(HTTP_PORT);
  
  socket.begin();
  socket.onEvent(handleWebSocket);

  Serial.print("Websockets Server listening on port: ");
  Serial.println(WEBSOCKETS_PORT);

  pinMode(MOTOR_A_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_A_THROTTLE_PIN, OUTPUT);

  pinMode(MOTOR_B_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_B_THROTTLE_PIN, OUTPUT);

  stop();

  Serial.println("Motors initialized");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK);

  pinMode(INTERRUPT_PIN, INPUT);

  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU initialized");
  } else {
    Serial.println("MPU failure");
  }
  
  mpu.dmpInitialize();
  mpu.setRate(DMP_SAMPLE_RATE);
  mpu.setDMPEnabled(true);
  mpu.setSleepEnabled(false);

  _dmp_packet_size = mpu.dmpGetFIFOPacketSize();

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

  Serial.println("DMP initialized");

  pinMode(BATTERY_PIN, INPUT);

  battery_timer.attach_ms(BATTERY_SAMPLE_RATE, checkBattery);
  tether_timer.attach_ms(TETHER_SAMPLE_RATE, checkTether);

  Serial.println("Ready");
}

/**
 * Main event loop.
 */
void loop() {
  if (_dmp_ready) {
    uint8_t mpu_int_status = mpu.getIntStatus();

    if (mpu_int_status & 0x10) {
      mpu.resetFIFO();
    } else if (mpu_int_status & 0x02) {
      updatePosition();
    }

    _dmp_ready = false;
  }

  server.handleClient();
  
  socket.loop();
}

/**
 * Handle the status request.
 */
void handleStatus() {
  StaticJsonDocument<200> doc;
  char buffer[200];

  doc["throttle"] = _throttle;
  doc["direction"] = _direction;
  doc["stopped"] = _stopped;
  doc["battery_level"] = _battery_level;
  
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
  StaticJsonDocument<200> doc;

  deserializeJson(doc, server.arg("plain"));

  if (!doc.containsKey("throttle")) {
    server.send(HTTP_UNPROCESSABLE_ENTITY);

    return;
  }

  setThrottle(doc["throttle"]);

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
 * Move in the forward direction.
 */
void forward() {
  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);

  _direction = "forward";

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
 * Disengage the motors and come to a complete stop.
 */
void stop() {
  analogWrite(MOTOR_A_THROTTLE_PIN, 0);
  analogWrite(MOTOR_B_THROTTLE_PIN, 0);

  _stopped = true;
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
 * MPU interrupt callback to signal DMP data is ready.
 */
void dmpDataReady() {
  _dmp_ready = true;
}

/**
 * Read the MPU data buffer and compute current orientation and acceleration.
 */
void updatePosition() {
  uint16_t fifo_count = mpu.getFIFOCount();

  while (fifo_count >= _dmp_packet_size) {
    mpu.getFIFOBytes(_dmp_fifo_buffer, _dmp_packet_size);

    fifo_count -= _dmp_packet_size;
  }

  mpu.dmpGetQuaternion(&q, _dmp_fifo_buffer);
  mpu.dmpGetAccel(&aa, _dmp_fifo_buffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
}

/**
 * Update the battery level and broadcast the event to any connected websockets clients.
 */
void checkBattery() {
  StaticJsonDocument<200> doc;
  char buffer[200];

  uint8_t raw = analogRead(BATTERY_PIN);

  _battery_level = raw / 1024.0;

  doc["name"] = "battery-update";
  doc["battery_level"] = _battery_level;

  serializeJson(doc, buffer);

  socket.broadcastTXT(buffer);
}

/**
 * Check that a client is connected or stop movement.
 */
void checkTether() {
  if (WiFi.softAPgetStationNum() == 0) {
    stop();
  }
}

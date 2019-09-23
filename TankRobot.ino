#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Ticker.h>
#include <Wire.h>
#include <FS.h>

#define AP_CHANNEL 1
#define MAX_CONNECTIONS 1
#define HIDDEN_SSID false

#define HTTP_PORT 80
#define HTTP_OK 200
#define HTTP_NOT_FOUND 404
#define HTTP_UNPROCESSABLE_ENTITY 422

#define MOTOR_A_DIRECTION_PIN 0
#define MOTOR_A_SPEED_PIN 5
#define MOTOR_B_DIRECTION_PIN 2
#define MOTOR_B_SPEED_PIN 4
#define FORWARD 1
#define REVERSE 0
#define MIN_SPEED 250
#define MAX_SPEED 1000

#define I2C_SDA_PIN 12
#define I2C_SCL_PIN 14
#define I2C_CLOCK 400000

#define MPU_ADDRESS 0x68
#define INTERRUPT_PIN 16
#define DMP_SAMPLE_RATE 50
#define LSB_SENSITIVITY 8192.0

#define TETHER_SAMPLE_RATE 250

#define SERIAL_BAUD 9600

ESP8266WebServer server(HTTP_PORT);

const IPAddress ip(192,168,4,1);
const IPAddress subnet(255,255,255,0);

char* _ssid = "Kirby The Tank Robot";
char* _password = "KeepSummerSafe";

unsigned int _speed = MIN_SPEED;

MPU6050 mpu(MPU_ADDRESS);

uint16_t _dmp_packet_size;
uint8_t _dmp_fifo_buffer[64];
bool _dmp_ready = false;

Ticker tether_timer;

void ICACHE_RAM_ATTR dmpDataReady();

/**
 * Bootstrap routine to run once.
 */
void setup() {
  Serial.begin(SERIAL_BAUD);

  if (SPIFFS.begin()) {
    Serial.println("SPIFFS mounted successfully");
  } else {
    Serial.println("Failed to mount SPIFFS");
  }
  
  WiFi.softAPConfig(ip, NULL, subnet);
  WiFi.softAP(_ssid, _password, AP_CHANNEL, HIDDEN_SSID, MAX_CONNECTIONS);

  Serial.println("Access point ready");
  Serial.print("SSID: ");
  Serial.println(_ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("MAC Address: ");
  Serial.println(WiFi.softAPmacAddress());

  server.serveStatic("/", SPIFFS, "/index.html");
  server.serveStatic("/settings", SPIFFS, "/index.html");
  server.serveStatic("/app.js", SPIFFS, "/app.js"); 
  server.serveStatic("/app.css", SPIFFS, "/app.css"); 
  server.serveStatic("/fa-solid-900.woff", SPIFFS, "/fa-solid-900.woff");
  server.serveStatic("/fa-solid-900.woff2", SPIFFS, "/fa-solid-900.woff2");
  server.serveStatic("/plucky.ogg", SPIFFS, "/plucky.ogg");
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/direction/forward", HTTP_PUT, handleForward);
  server.on("/api/direction/reverse", HTTP_PUT, handleReverse);
  server.on("/api/direction/left", HTTP_PUT, handleLeft);
  server.on("/api/direction/right", HTTP_PUT, handleRight);
  server.on("/api/movement/go", HTTP_PUT, handleGo);
  server.on("/api/movement/stop", HTTP_PUT, handleStop);
  server.on("/api/movement/speed", HTTP_PUT, handleSetSpeed);
  server.onNotFound(handleNotFound);

  server.begin();

  Serial.print("HTTP Server listening on port: ");
  Serial.println(HTTP_PORT);

  pinMode(MOTOR_A_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_A_SPEED_PIN, OUTPUT);

  pinMode(MOTOR_B_DIRECTION_PIN, OUTPUT);
  pinMode(MOTOR_B_SPEED_PIN, OUTPUT);

  forward();
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

  tether_timer.attach_ms(TETHER_SAMPLE_RATE, checkTether);

  Serial.println("Ready");
}

/**
 * Main event loop.
 */
void loop() {
  server.handleClient();
}

/**
 * Handle the status request.
 */
void handleStatus() {
  StaticJsonDocument<500> doc;
  char buffer[500];

  doc["speed"] = _speed;
  
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
 * Handle the go movement request.
 */
void handleGo() {
  go();

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
void handleSetSpeed() {
  StaticJsonDocument<500> doc;

  deserializeJson(doc, server.arg("plain"));

  if (!doc.containsKey("speed")) {
    server.send(HTTP_UNPROCESSABLE_ENTITY);

    return;
  }

  setSpeed(doc["speed"]);

  server.send(HTTP_OK);
}

/**
 * Catch all web route to return a 404 response if a route not found.
 */
int handleNotFound() {
  server.send(HTTP_NOT_FOUND, "text/html", "<h1>Page not found</h1>");
}

/**
 * Move in the forward direction.
 */
void forward() {
  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);
}

/**
 * Turn left.
 */
void left() {
  digitalWrite(MOTOR_A_DIRECTION_PIN, FORWARD);
  digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);
}

/**
 * Turn right.
 */
void right() {
  digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
  digitalWrite(MOTOR_B_DIRECTION_PIN, FORWARD);
}

/**
 * Move in reverse.
 */
void reverse() {
  digitalWrite(MOTOR_A_DIRECTION_PIN, REVERSE);
  digitalWrite(MOTOR_B_DIRECTION_PIN, REVERSE);
}

/**
 * Engage the motors and move in a direction at a particular speed.
 */
void go() {
  analogWrite(MOTOR_A_SPEED_PIN, _speed);
  analogWrite(MOTOR_B_SPEED_PIN, _speed);
}

/**
 * Disengage the motors and come to a complete stop.
 */
void stop() {
  analogWrite(MOTOR_A_SPEED_PIN, 0);
  analogWrite(MOTOR_B_SPEED_PIN, 0);
}

/**
 * Set the speed of the motors.
 * 
 * @param int speed
 */
void setSpeed(int speed) {
  _speed = max(MIN_SPEED, min(speed, MAX_SPEED)); 
}

/**
 * Check that the client is connected or stop.
 */
void checkTether() {
  if (WiFi.softAPgetStationNum() == 0) {
    stop();
  }
}

/**
 * MPU interrupt callback to signal DMP data is ready.
 */
void dmpDataReady() {
  _dmp_ready = true;
}

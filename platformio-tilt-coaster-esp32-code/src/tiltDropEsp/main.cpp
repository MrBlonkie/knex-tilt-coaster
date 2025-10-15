#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

// === WiFi Config ===
const char* ssid = "wieditleestisZOT";
const char* password = "jemama123";

// === Server ===
WebServer server(80);

// === Tilt-track motor config ===
#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23
#define STEPS_TILT_TRACK 550

AccelStepper tiltTrackStepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

// === Servo config ===
#define SERVO_PIN 25
Servo releaseServo;

// === Hall sensors ===
#define hallSensorOnTiltdrop     32
#define hallSensorTiltdropClosed 27
#define hallSensorTiltdropOpen   34
#define hallSensorOffTiltdrop    26

bool hallClosed = false;
bool hallOpen = false;
bool coasterOnTiltdrop = false;

// === Servo drop state machine ===
enum DropState { DROP_IDLE, DROPPING, RESETTING };
DropState dropState = DROP_IDLE;
unsigned long lastServoMillis = 0;
int servoPos = 0;
bool startServoDrop = false;

// === Functions ===
void updateSensors() {
    hallClosed = digitalRead(hallSensorTiltdropClosed) == LOW;
    hallOpen   = digitalRead(hallSensorTiltdropOpen) == LOW;
    coasterOnTiltdrop = digitalRead(hallSensorOnTiltdrop) == LOW;
}

void runServoDrop() {
    unsigned long now = millis();
    if (dropState == DROP_IDLE) return;

    if (dropState == DROPPING && now - lastServoMillis > 10) {
        lastServoMillis = now;
        if (servoPos < 90) {
            servoPos++;
            releaseServo.write(servoPos);
        } else {
            dropState = RESETTING;
            lastServoMillis = now;
        }
    } else if (dropState == RESETTING && now - lastServoMillis > 20) {
        lastServoMillis = now;
        if (servoPos > 0) {
            servoPos--;
            releaseServo.write(servoPos);
        } else {
            dropState = DROP_IDLE;
            startServoDrop = false;
        }
    }
}

// === Web endpoints ===
void handleTiltOpen() {
    tiltTrackStepper.move(-STEPS_TILT_TRACK); // open
    server.send(200, "application/json", "{\"tiltdrop\":\"opening\"}");
}

void handleTiltClose() {
    tiltTrackStepper.move(STEPS_TILT_TRACK); // close
    server.send(200, "application/json", "{\"tiltdrop\":\"closing\"}");
}

void handleTiltDrop() {
    if (coasterOnTiltdrop && hallClosed) {
        startServoDrop = true;
        dropState = DROPPING;
        server.send(200, "application/json", "{\"tiltdrop\":\"dropping\"}");
    } else {
        server.send(400, "application/json", "{\"error\":\"Tiltdrop not ready\"}");
    }
}

void handleTiltStatus() {
    String json = "{";
    json += "\"closed\":" + String(hallClosed ? "true" : "false");
    json += ",\"open\":" + String(hallOpen ? "true" : "false");
    json += ",\"coasterOn\":" + String(coasterOnTiltdrop ? "true" : "false");
    json += "}";
    server.send(200, "application/json", json);
}

// === Setup ===
void setup() {
    Serial.begin(115200);

    pinMode(hallSensorOnTiltdrop, INPUT);
    pinMode(hallSensorTiltdropClosed, INPUT);
    pinMode(hallSensorTiltdropOpen, INPUT);
    pinMode(hallSensorOffTiltdrop, INPUT);

    tiltTrackStepper.setMaxSpeed(200.0);
    tiltTrackStepper.setAcceleration(100.0);

    releaseServo.attach(SERVO_PIN);
    releaseServo.write(0); // servo startpositie

    // WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("Connected! IP: " + WiFi.localIP().toString());

    // Routes
    server.on("/tiltdrop/open", handleTiltOpen);
    server.on("/tiltdrop/close", handleTiltClose);
    server.on("/tiltdrop/drop", handleTiltDrop);
    server.on("/tiltdrop/status", handleTiltStatus);

    server.begin();
    Serial.println("TiltDrop ESP ready!");
}

// === Loop ===
void loop() {
    server.handleClient();
    updateSensors();
    tiltTrackStepper.run();
    runServoDrop();
}

#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <FastLED.h>
#include "../shared/env.h"

// === Server ===
WebServer server(80);

// === Tilt-track motor config ===
#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23
#define STEPS_TILT_TRACK 550

AccelStepper tiltTrackStepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);
const int stepsTiltTrack = 560;
bool tiltdropMotorState = false;

// === Servo config ===
#define SERVO_PIN 25
Servo releaseServo;
bool releasedropMotorState = false;
bool tiltdropMotorMoving = false;

// === LED Config === 
#define NUM_LEDS_TILT 3 
#define LEDS_TILT_PIN 21 
CRGB tiltLeds[NUM_LEDS_TILT];
// Non-blocking timing
unsigned long previousMillis = 0;
unsigned long effectInterval = 50; // snelheid van het effect
CRGB colors[3] = {CRGB::Red, CRGB::Purple, CRGB::Green};
// Index en richting voor elke LED
int colorIndex[NUM_LEDS_TILT] = {0, 1, 2};
int brightness[NUM_LEDS_TILT] = {255, 200, 150};
int step[NUM_LEDS_TILT] = {15, 10, 20}; // stapjes voor fade

// === Sensors ===
#define hallSensorOnTiltdrop 32
#define hallSensorTiltdropClosed 27
#define hallSensorTiltdropOpen 34
#define hallSensorOffTiltdrop 26

bool hallSensorOnTiltdropState = false;
bool hallSensorTiltdropClosedState = false;
bool hallSensorTiltdropOpenState = false;
bool hallSensorOffTiltdropState = false;

bool hallSensorOnTiltdropStateWeb = false;
bool hallSensorTiltdropClosedStateWeb = false;
bool hallSensorTiltdropOpenStateWeb = false;
bool hallSensorOffTiltdropStateWeb = false;

// === STATES ===
enum CoasterState
{
    STATE_IDLE,
    STATE_NOT_AT_TILTDROP,
    STATE_ENTER_TILTDROP,
    STATE_DROP,
    STATE_RESET_DROP,
    STATE_ERROR
};

CoasterState currentState = STATE_IDLE;
String currentStateName = "IDLE";

void setState(CoasterState newState)
{
    currentState = newState;
    switch (newState)
    {
    case STATE_IDLE:
        currentStateName = "IDLE";
        break;
    case STATE_NOT_AT_TILTDROP:
        currentStateName = "NOT_AT_TILTDROP";
        break;
    case STATE_ENTER_TILTDROP:
        currentStateName = "ENTER_TILTDROP";
        break;
    case STATE_DROP:
        currentStateName = "DROP";
        break;
    case STATE_RESET_DROP:
        currentStateName = "RESET_DROP";
        break;
    case STATE_ERROR:
        currentStateName = "ERROR";
        break;
    }
    Serial.println("[STATE] → " + currentStateName);
}

// === State control vars ===
bool coasterDispatched = false;
bool manualMode = false;
bool tiltdropMotorManual = false;

// === FUNCTIONS ===
void updateTiltdropSensors()
{
    bool onTiltdrop = digitalRead(hallSensorOnTiltdrop) == LOW;
    bool tiltdropClosed = digitalRead(hallSensorTiltdropClosed) == LOW;
    bool tiltdropOpen = digitalRead(hallSensorTiltdropOpen) == LOW;
    bool offTiltdrop = digitalRead(hallSensorOffTiltdrop) == LOW;

    if (onTiltdrop != hallSensorOnTiltdropState ||
        tiltdropClosed != hallSensorTiltdropClosedState ||
        tiltdropOpen != hallSensorTiltdropOpenState ||
        offTiltdrop != hallSensorOffTiltdropState)
    {

        hallSensorOnTiltdropState = onTiltdrop;
        hallSensorTiltdropClosedState = tiltdropClosed;
        hallSensorTiltdropOpenState = tiltdropOpen;
        hallSensorOffTiltdropState = offTiltdrop;
    }
}

void LedTiltEffect() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= effectInterval) {
    previousMillis = currentMillis;

    for(int i = 0; i < NUM_LEDS_TILT; i++) {
      // LED faden richting helderste of zwakste waarde
      tiltLeds[i] = colors[colorIndex[i]];
      tiltLeds[i].nscale8_video(brightness[i]);

      // Puls / fade effect
      brightness[i] -= step[i];
      if(brightness[i] <= 50 || brightness[i] >= 255) {
        step[i] = -step[i]; // draai richting om
        // eventueel naar volgende kleur
        if(step[i] > 0) {
          colorIndex[i] = (colorIndex[i] + 1) % 3;
        }
      }
    }

    FastLED.show();
  }
}

// Helper functie motor stop (dit moet op deze manier omdat gewoon een stop() doen met deze library de motor laat uitlopen en niet onmiddelijk stopt)
void StopStepperMotor(AccelStepper &motor)
{
    motor.setCurrentPosition(motor.currentPosition());
    motor.moveTo(motor.currentPosition());
}

// === AUTO LOGIC ===
// TODO
//
//
//

// === SETUP ===
void setup()
{
    Serial.begin(115200);

    //LED 
    FastLED.addLeds<NEOPIXEL, LEDS_TILT_PIN>(tiltLeds, NUM_LEDS_TILT);
    FastLED.clear();

    pinMode(hallSensorOnTiltdrop, INPUT_PULLUP);
    pinMode(hallSensorOffTiltdrop, INPUT_PULLUP);
    pinMode(hallSensorTiltdropClosed, INPUT_PULLUP);
    pinMode(hallSensorTiltdropOpen, INPUT_PULLUP);

    tiltTrackStepper.setMaxSpeed(500.0);
    tiltTrackStepper.setSpeed(400.0);
    tiltTrackStepper.setAcceleration(200.0);

    releaseServo.attach(25);

    // Wifi Connect
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(300);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("Connected! IP: " + WiFi.localIP().toString());

   
    // Manual Control Endpoints
    server.on("/manual/on", []()
              {
    manualMode = true;
    setState(STATE_IDLE);
    server.send(200, "application/json", "{\"manual\":\"enabled\"}");
    Serial.println("[MANUAL] Manual mode enabled"); });

    server.on("/manual/off", []()
              {
    manualMode = false;
    StopStepperMotor(tiltTrackStepper);
    tiltdropMotorManual = false;
    setState(STATE_IDLE);
    server.send(200, "application/json", "{\"manual\":\"disabled\"}");
    Serial.println("[MANUAL] Manual mode disabled"); });

    //tiltdrop stepper
    server.on("/manual/tiltdropmotor/open", []() {
    if (manualMode) {
      Serial.println("opening tiltdrop...");
      tiltTrackStepper.move(-stepsTiltTrack);
      tiltdropMotorState = true;
      tiltdropMotorMoving = true;
      Serial.println("tiltrack now open");
    }
    server.send(200, "application/json", "{\"stationmotor\":\"on\"}");
    Serial.println("[MANUAL] TILT motor OPEN");
  });

  server.on("/manual/tiltdropmotor/close", []() {
    if (manualMode) {
      Serial.println("closing tiltdrop...");
      tiltTrackStepper.move(stepsTiltTrack);
      tiltdropMotorState = false;
      tiltdropMotorMoving = true; 
      Serial.println("tiltrack now closed");
    }
    server.send(200, "application/json", "{\"stationmotor\":\"on\"}");
    Serial.println("[MANUAL] TILT motor CLOSE");
  });

  //releasedrop servo
  server.on("/manual/releasedropmotor/open", []() {
    if (manualMode) {
      Serial.println("opening release...");
        for(int posDegrees = 90; posDegrees >= 0; posDegrees--) {
            releaseServo.write(posDegrees);
            Serial.println(posDegrees);
        }
      releasedropMotorState = true;
      Serial.println("release now open");
    }
    server.send(200, "application/json", "{\"stationmotor\":\"on\"}");
    Serial.println("[MANUAL] Release motor OPEN");
  });

  server.on("/manual/releasedropmotor/close", []() {
    if (manualMode) {
      Serial.println("closing release...");
      for(int posDegrees = 0; posDegrees <= 90; posDegrees++) {
            releaseServo.write(posDegrees);
            Serial.println(posDegrees);
        }
      releasedropMotorState = false;
      Serial.println("release now closed");
    }
    server.send(200, "application/json", "{\"stationmotor\":\"on\"}");
    Serial.println("[MANUAL] Release motor CLOSE");
  });

    server.begin();
    setState(STATE_IDLE);
  
}

void loop()
{
    server.handleClient();
    updateTiltdropSensors();

    if (tiltdropMotorMoving) {
        LedTiltEffect();
        bool stillRunning = tiltTrackStepper.run(); // true zolang de motor nog beweegt
        if (!stillRunning) {                        // als hij klaar is
            tiltdropMotorMoving = false;            // beweging stop
            Serial.println("[MANUAL] TILT motor finished movement");
        }
    }
}
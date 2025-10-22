#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <FastLED.h>
#include "LedSetup.h"
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
#define LEDS_TILT_PIN 13 
CRGB tiltLeds[NUM_LEDS_TILT];
// Non-blocking timing
unsigned long previousMillis = 0;
unsigned long effectInterval = 50; // snelheid van het effect
CRGB colors[3] = {CRGB::Red, CRGB::Purple, CRGB::Green};
// Index en richting voor elke LED
int colorIndex[NUM_LEDS_TILT] = {0, 1, 2};
int brightness[NUM_LEDS_TILT] = {255, 200, 150};
int step[NUM_LEDS_TILT] = {15, 10, 20}; // stapjes voor fade

// === LED VARIABLES ===
CRGB currentColor = CRGB::Blue;   // huidige kleur voor fade
CRGB targetColor = CRGB::Blue;    // kleur waar we naar toe faden

int fadeBrightness = 50;           // puls brightness
bool fadeIncreasing = true;        // puls richting
unsigned long lastFadeMillis = 0;
int fadeInterval = 30;             // snelheid van fade

// === FLASH VARIABLES ===
unsigned long flashStartMillis = 0;
int flashCount = 0;
bool flashing = false;

// === RED PULSE VARIABLES ===
int redBrightness = 50;
bool redIncreasing = true;

// === HELPER: SET ALL LEDS ===
void setAllLeds(CRGB color, int brightnessVal) {
    for (int i = 0; i < NUM_LEDS_TILT; i++) {
        tiltLeds[i] = color;
        tiltLeds[i].nscale8_video(brightnessVal);
    }
    FastLED.show();
}

// === TARGET COLOR FUNCTIE ===
void SetTargetColor(CRGB newColor) {
    targetColor = newColor;
}

// === SMOOTH COLOR FADE + PULSE ===
void UpdateLedFade() {
    unsigned long now = millis();
    if (now - lastFadeMillis < fadeInterval) return;
    lastFadeMillis = now;

    // Smooth fade currentColor → targetColor
    currentColor.r = currentColor.r + ((targetColor.r - currentColor.r) / 5);
    currentColor.g = currentColor.g + ((targetColor.g - currentColor.g) / 5);
    currentColor.b = currentColor.b + ((targetColor.b - currentColor.b) / 5);

    // Pulsing / fade brightness
    if (fadeIncreasing) fadeBrightness += 5;
    else fadeBrightness -= 5;
    if (fadeBrightness >= 150) fadeIncreasing = false;
    if (fadeBrightness <= 50) fadeIncreasing = true;

    setAllLeds(currentColor, fadeBrightness);
}

// === WHITE FLASHES ===
bool LedWhiteFlash(int numFlashes, int intervalMs) {
    unsigned long now = millis();
    if (!flashing) {
        flashStartMillis = now;
        flashCount = 0;
        flashing = true;
    }

    if (now - flashStartMillis >= intervalMs) {
        flashStartMillis = now;
        flashCount++;

        if (flashCount % 2 == 1) setAllLeds(CRGB::White, 255);
        else setAllLeds(CRGB::Black, 0);

        if (flashCount >= numFlashes * 2) {
            flashing = false;
            return true; // klaar
        }
    }
    return false; // nog bezig
}

// === FAST RED PULSE ===
void LedRedPulseFast() {
    unsigned long now = millis();
    if (now - lastFadeMillis < 30) return;
    lastFadeMillis = now;

    if (redIncreasing) redBrightness += 20;
    else redBrightness -= 20;
    if (redBrightness >= 255) redIncreasing = false;
    if (redBrightness <= 50) redIncreasing = true;

    setAllLeds(CRGB::Red, redBrightness);
}

// === MASTER UPDATE LED FUNCTION ===
void UpdateLeds() {
    static bool whiteFlashesDone = false;

    if (!tiltdropMotorMoving) {
        // Motor niet actief → idle blauw
        whiteFlashesDone = false; // reset flash
        SetTargetColor(CRGB::Blue);
        UpdateLedFade();
    } else {
        if (!whiteFlashesDone) {
            whiteFlashesDone = LedWhiteFlash(3, 150); // 3 flashes, 150ms
        } else {
            LedRedPulseFast(); // snel rood
        }
    }
}




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

void loop() {
    server.handleClient();
    updateTiltdropSensors();
    
    // Motor beweging
    if (tiltdropMotorMoving) {
        bool stillRunning = tiltTrackStepper.run(); // true zolang de motor nog beweegt
        if (!stillRunning) {
            tiltdropMotorMoving = false;
            Serial.println("[MANUAL] TILT motor finished movement");
        }
    }

    // LED update
    UpdateLeds();
}

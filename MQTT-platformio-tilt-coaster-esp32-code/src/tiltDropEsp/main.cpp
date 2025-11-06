#include <WiFi.h>
#include <PubSubClient.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <FastLED.h>
#include "LedSetup.h"
#include "../shared/env.h" // ssid, password, mqtt_server, mqtt_port

// === WiFi & MQTT ===
WiFiClient espClient;
PubSubClient client(espClient);

// === Tilt-track motor config ===
#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23
#define STEPS_TILT_TRACK 550
AccelStepper tiltTrackStepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);
bool tiltdropMotorState = false;
bool tiltdropMotorMoving = false;

// === Servo config ===
#define SERVO_PIN 25
Servo releaseServo;
bool releasedropMotorState = false;

// === LED Config ===
#define NUM_LEDS_TILT 3
#define LEDS_TILT_PIN 13
CRGB tiltLeds[NUM_LEDS_TILT];
unsigned long previousMillis = 0;
unsigned long effectInterval = 50;
CRGB colors[3] = {CRGB::Red, CRGB::Purple, CRGB::Green};
int colorIndex[NUM_LEDS_TILT] = {0, 1, 2};
int brightness[NUM_LEDS_TILT] = {255, 200, 150};
int step[NUM_LEDS_TILT] = {15, 10, 20};

CRGB currentColor = CRGB::Blue;
CRGB targetColor = CRGB::Blue;
int fadeBrightness = 50;
bool fadeIncreasing = true;
unsigned long lastFadeMillis = 0;
int fadeInterval = 30;

unsigned long flashStartMillis = 0;
int flashCount = 0;
bool flashing = false;

int redBrightness = 50;
bool redIncreasing = true;

// === Sensors ===
#define hallSensorOnTiltdrop 32
#define hallSensorTiltdropClosed 27
#define hallSensorTiltdropOpen 34
#define hallSensorOffTiltdrop 26

bool hallSensorOnTiltdropState = false;
bool hallSensorTiltdropClosedState = false;
bool hallSensorTiltdropOpenState = false;
bool hallSensorOffTiltdropState = false;

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
bool manualMode = false;
bool tiltdropMotorManual = false;

// === LED HELPERS ===
void setAllLeds(CRGB color, int brightnessVal)
{
    for (int i = 0; i < NUM_LEDS_TILT; i++)
    {
        tiltLeds[i] = color;
        tiltLeds[i].nscale8_video(brightnessVal);
    }
    FastLED.show();
}

void SetTargetColor(CRGB newColor) { targetColor = newColor; }

void UpdateLedFade()
{
    unsigned long now = millis();
    if (now - lastFadeMillis < fadeInterval)
        return;
    lastFadeMillis = now;

    currentColor.r += (targetColor.r - currentColor.r) / 5;
    currentColor.g += (targetColor.g - currentColor.g) / 5;
    currentColor.b += (targetColor.b - currentColor.b) / 5;

    if (fadeIncreasing)
        fadeBrightness += 5;
    else
        fadeBrightness -= 5;
    if (fadeBrightness >= 150)
        fadeIncreasing = false;
    if (fadeBrightness <= 50)
        fadeIncreasing = true;

    setAllLeds(currentColor, fadeBrightness);
}

bool LedWhiteFlash(int numFlashes, int intervalMs)
{
    unsigned long now = millis();
    static bool localFlashing = false;
    static int localFlashCount = 0;
    static unsigned long localStart = 0;

    if (!localFlashing)
    {
        localStart = now;
        localFlashCount = 0;
        localFlashing = true;
    }

    if (now - localStart >= intervalMs)
    {
        localStart = now;
        localFlashCount++;
        if (localFlashCount % 2 == 1)
            setAllLeds(CRGB::White, 255);
        else
            setAllLeds(CRGB::Black, 0);
        if (localFlashCount >= numFlashes * 2)
        {
            localFlashing = false;
            return true;
        }
    }
    return false;
}

void LedRedPulseFast()
{
    unsigned long now = millis();
    if (now - lastFadeMillis < 30)
        return;
    lastFadeMillis = now;

    if (redIncreasing)
        redBrightness += 20;
    else
        redBrightness -= 20;
    if (redBrightness >= 255)
        redIncreasing = false;
    if (redBrightness <= 50)
        redIncreasing = true;

    setAllLeds(CRGB::Red, redBrightness);
}

void UpdateLeds()
{
    static bool whiteFlashesDone = false;

    if (!tiltdropMotorMoving)
    {
        whiteFlashesDone = false;
        SetTargetColor(CRGB::Blue);
        UpdateLedFade();
    }
    else
    {
        if (!whiteFlashesDone)
        {
            whiteFlashesDone = LedWhiteFlash(3, 150);
        }
        else
            LedRedPulseFast();
    }
}

// === Sensors update ===
void updateTiltdropSensors()
{
    bool onTiltdrop = digitalRead(hallSensorOnTiltdrop) == LOW;
    bool tiltdropClosed = digitalRead(hallSensorTiltdropClosed) == LOW;
    bool tiltdropOpen = digitalRead(hallSensorTiltdropOpen) == LOW;
    bool offTiltdrop = digitalRead(hallSensorOffTiltdrop) == LOW;

    hallSensorOnTiltdropState = onTiltdrop;
    hallSensorTiltdropClosedState = tiltdropClosed;
    hallSensorTiltdropOpenState = tiltdropOpen;
    hallSensorOffTiltdropState = offTiltdrop;
}

// === Helper motor stop ===
void StopStepperMotor(AccelStepper &motor)
{
    motor.setCurrentPosition(motor.currentPosition());
    motor.moveTo(motor.currentPosition());
}

// === MQTT callback ===
void callback(char *topic, byte *payload, unsigned int length)
{
    String message = "";
    for (int i = 0; i < length; i++)
        message += (char)payload[i];

    if (String(topic) == "tiltdrop/manual")
    {
        manualMode = (message == "on");
        setState(STATE_IDLE);
        if (!manualMode)
            tiltdropMotorManual = false;
    }
    else if (String(topic) == "tiltdrop/tiltdropmotor" && message == "open")
    {
        if (manualMode)
        {
            tiltTrackStepper.move(-STEPS_TILT_TRACK);
            tiltdropMotorState = true;
            tiltdropMotorMoving = true;
            tiltdropMotorManual=true;
        }
    }
    else if (String(topic) == "tiltdrop/tiltdropmotor" && message == "close")
    {
        if (manualMode)
        {
            tiltTrackStepper.move(STEPS_TILT_TRACK);
            tiltdropMotorState = false;
            tiltdropMotorMoving = true;
            tiltdropMotorManual=true;
        }
    }
    else if (String(topic) == "tiltdrop/releasedropmotor" && message == "open")
    {
        if (manualMode)
        {
            for (int pos = 90; pos >= 0; pos--)
                releaseServo.write(pos);
            releasedropMotorState = true;
        }
    }
    else if (String(topic) == "tiltdrop/releasedropmotor" && message == "close")
    {
        if (manualMode)
        {
            for (int pos = 0; pos <= 90; pos++)
                releaseServo.write(pos);
            releasedropMotorState = false;
        }
    }
}

// === MQTT connect ===
void connectMQTT()
{
    while (!client.connected())
    {
        Serial.print("Connecting MQTT...");
        if (client.connect("tiltdropESP32"))
        {
            Serial.println("connected");
            client.subscribe("tiltdrop/manual");
            client.subscribe("tiltdrop/tiltdropmotor");
            client.subscribe("tiltdrop/releasedropmotor");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.println(client.state());
            delay(2000);
        }
    }
}

// === Publish Status via MQTT ===
String lastStatus = "";

void publishStatusIfChanged()
{
    String status = "{";
    status += "\"hallSensorOnTiltdrop\":" + String(hallSensorOnTiltdropState ? "true" : "false");
    status += ",\"hallSensorTiltdropClosed\":" + String(hallSensorTiltdropClosedState ? "true" : "false");
    status += ",\"hallSensorTiltdropOpen\":" + String(hallSensorTiltdropOpenState ? "true" : "false");
    status += ",\"hallSensorOffTiltdrop\":" + String(hallSensorOffTiltdropState ? "true" : "false");
    status += ",\"tiltdropMotorMoving\":" + String(tiltdropMotorMoving ? "true" : "false");
    status += ",\"manualMode\":" + String(manualMode ? "true" : "false");
    status += ",\"releasedropMotorState\":" + String(releasedropMotorState ? "true" : "false");
    status += ",\"currentState\":\"" + currentStateName + "\"";
    status += "}";

    if (status != lastStatus)
    {
        Serial.println("[STATUS JSON] " + status);
        client.publish("tiltdrop/status", status.c_str());
        lastStatus = status;
    }
}

// === Setup ===
void setup()
{
    Serial.begin(115200);

    // WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(300);
        Serial.print(".");
    }
    Serial.println("Connected! IP: " + WiFi.localIP().toString());

    // MQTT
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    connectMQTT();

    // LEDs
    FastLED.addLeds<NEOPIXEL, LEDS_TILT_PIN>(tiltLeds, NUM_LEDS_TILT);

    // Sensors
    pinMode(hallSensorOnTiltdrop, INPUT_PULLUP);
    pinMode(hallSensorTiltdropClosed, INPUT_PULLUP);
    pinMode(hallSensorTiltdropOpen, INPUT_PULLUP);
    pinMode(hallSensorOffTiltdrop, INPUT_PULLUP);

    // Motor
    tiltTrackStepper.setMaxSpeed(500.0);
    tiltTrackStepper.setAcceleration(200.0);

    // Servo
    releaseServo.attach(SERVO_PIN);

    setState(STATE_IDLE);
}

// === Loop ===
void loop()
{
    if (!client.connected())
        connectMQTT();
    client.loop();

    updateTiltdropSensors();


    if (tiltdropMotorMoving)
    {
        bool stillRunning = tiltTrackStepper.run();
        if (!stillRunning)
        {
            tiltdropMotorMoving = false;
            Serial.println("[MANUAL] Tilt motor movement finished");
        }
    }

    UpdateLeds();

    // Status publish
    publishStatusIfChanged();
}

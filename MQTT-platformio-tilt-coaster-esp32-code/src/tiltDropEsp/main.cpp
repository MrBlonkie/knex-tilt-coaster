#include <WiFi.h>
#include <PubSubClient.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <FastLED.h>
#include "LedSetup.h"      // Zorg dat de nieuwe versie hierboven gebruikt wordt
#include "../shared/env.h" // ssid, password, mqtt_server, mqtt_portt

// === WiFi & MQTT ===
WiFiClient espClient;
PubSubClient client(espClient);
const char *deviceName = "tiltdrop";
String clientId;
const char *connectivityTopic = "rollercoaster/tiltdrop/status";

// === Heartbeat Config ===
unsigned long lastHeartbeat = 0;
const long heartbeatInterval = 2500;

// === Tilt-track motor config ===
#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23
AccelStepper tiltTrackStepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

#define STEPS_UNTIL_SLOWDOWN 400
#define STEPS_OVERRUN 40
#define MOTOR_SPEED_FAST 450.0
#define MOTOR_SPEED_SLOW 100.0

enum MotorRunState
{
    IDLE,
    SEEKING,
    OVERRUN
};
MotorRunState motorRunState = IDLE;
long motorReferencePosition = 0;
bool motorInSlowZone = false;

bool tiltdropMotorState = false;
bool tiltdropMotorMoving = false;
bool isTiltdropTrackOpen = false;
bool coasterDispatched = false;

// === Servo config ===
#define SERVO_PIN 25
Servo releaseServo;
bool releasedropMotorState = false;

// === Sensors ===
#define hallSensorOnTiltdrop 34
#define hallSensorTiltdropClosed 26
#define hallSensorTiltdropOpen 32
#define hallSensorOffTiltdrop 27

bool hallSensorOnTiltdropState = false;
bool hallSensorTiltdropClosedState = false;
bool hallSensorTiltdropOpenState = false;
bool hallSensorOffTiltdropState = false;

bool ledDropControlState = false;

bool tiltdropReady = false; // Is true wanneer trein op lifthill is (onderweg naar tilt)
bool autoMode = false;

// === STATES ===
enum CoasterState
{
    STATE_IDLE,
    STATE_ON_TILTDROP,
    STATE_TILTDROP_OPEN,
    STATE_RESET_TILTDROP,
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
    case STATE_ON_TILTDROP:
        currentStateName = "ON_TILTDROP";
        break;
    case STATE_TILTDROP_OPEN:
        currentStateName = "TILTDROP_OPEN";
        break;
    case STATE_RESET_TILTDROP:
        currentStateName = "RESET_TILTDROP";
        break;
    case STATE_ERROR:
        currentStateName = "ERROR";
        break;
    }
    Serial.println("[STATE] -> " + currentStateName);
}

// === State control vars ===
bool manualMode = false;
bool tiltdropMotorManual = false;

void updateTiltdropSensors()
{
    hallSensorOnTiltdropState = (digitalRead(hallSensorOnTiltdrop) == LOW);
    hallSensorTiltdropClosedState = (digitalRead(hallSensorTiltdropClosed) == LOW);
    hallSensorTiltdropOpenState = (digitalRead(hallSensorTiltdropOpen) == LOW);
    hallSensorOffTiltdropState = (digitalRead(hallSensorOffTiltdrop) == LOW);
}

// === LED CONTROL LOOP ===
void UpdateLeds()
{
    static bool whiteFlashesDone = false;

    // 1. TILT LEDS LOGIC
    if (!tiltdropMotorMoving)
    {
        whiteFlashesDone = false;
        UpdateTiltIdle(); // Blauw ademen
    }
    else
    {
        if (!whiteFlashesDone)
        {
            whiteFlashesDone = UpdateTiltFlash(3, 150);
        }
        else
        {
            UpdateTiltRedPulse();
        }
    }

    // 2. DROP LEDS LOGIC
    // Als de offtiltdrop sensor (einde drop) getriggerd wordt -> Rood/Wit Alarm
    // Anders -> Groen Ademen
    UpdateDropLeds(ledDropControlState);

    // 3. TOWER LEDS LOGIC
    // Als tiltdropReady true is, betekent dit dat de trein op de lifthill is (onderweg naar ons)
    UpdateTowerLeds(tiltdropReady);

    // Show voor alle strips in één keer
    FastLED.show();
}

// === MQTT callback ===
void callback(char *topic, byte *payload, unsigned int length)
{
    String message = "";
    for (int i = 0; i < length; i++)
        message += (char)payload[i];

    if (String(topic) == "tiltdrop/manual" && message == "on")
    {
        manualMode = true;
        setState(STATE_IDLE);
        
    }
    else if (String(topic) == "tiltdrop/manual" && message == "off")
    {
        manualMode = false;
        tiltdropMotorManual = false;
        // close tiltdropmotor if not closed
        if (hallSensorTiltdropClosedState)
        {
            Serial.println("[MANUAL] Motor is al dicht.");
        }
        else
        {
            // Vertel de motor om een (bijna) oneindig aantal stappen 'dicht' (positief) te zetten
            tiltTrackStepper.move(9999999);
            tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_FAST); // Start op volle snelheid

            tiltdropMotorState = false; // false = we zijn aan het sluiten
            tiltdropMotorMoving = true;
            tiltdropMotorManual = true;

            // Stel de state machine in
            motorReferencePosition = tiltTrackStepper.currentPosition(); // Onthoud startpositie
            motorRunState = SEEKING;
            motorInSlowZone = false; // We zijn nog niet in de slow zone

            Serial.println("[MANUAL] Start motor: SLUITEN (SEEKING)");
        }

        // close servo if not closed
        if (releasedropMotorState)
        {
            for (int pos = 0; pos <= 90; pos++)
                releaseServo.write(pos);
            releasedropMotorState = false;
        }
    }

    else if (String(topic) == "tiltdrop/tiltdropmotor" && message == "open")
    {
        if (manualMode)
        {
            // Controleer of we niet al open zijn
            if (hallSensorTiltdropOpenState)
            {
                Serial.println("[MANUAL] Motor is al open.");
                return; // Doe niets
            }
            // Vertel de motor om een (bijna) oneindig aantal stappen 'open' (negatief) te zetten
            tiltTrackStepper.move(-9999999);
            tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_FAST); // Start op volle snelheid

            tiltdropMotorState = true; // true = we zijn aan het openen
            tiltdropMotorMoving = true;
            tiltdropMotorManual = true;

            // Stel de state machine in
            motorReferencePosition = tiltTrackStepper.currentPosition(); // Onthoud startpositie
            motorRunState = SEEKING;
            motorInSlowZone = false; // We zijn nog niet in de slow zone

            Serial.println("[MANUAL] Start motor: OPENEN (SEEKING)");
        }
    }
    else if (String(topic) == "tiltdrop/tiltdropmotor" && message == "close")
    {
        if (manualMode)
        {
            // Controleer of we niet al dicht zijn
            if (hallSensorTiltdropClosedState)
            {
                Serial.println("[MANUAL] Motor is al dicht.");
                return; // Doe niets
            }
            // Vertel de motor om een (bijna) oneindig aantal stappen 'dicht' (positief) te zetten
            tiltTrackStepper.move(9999999);
            tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_FAST); // Start op volle snelheid

            tiltdropMotorState = false; // false = we zijn aan het sluiten
            tiltdropMotorMoving = true;
            tiltdropMotorManual = true;

            // Stel de state machine in
            motorReferencePosition = tiltTrackStepper.currentPosition(); // Onthoud startpositie
            motorRunState = SEEKING;
            motorInSlowZone = false; // We zijn nog niet in de slow zone

            Serial.println("[MANUAL] Start motor: SLUITEN (SEEKING)");
        }
    }    
    else if (String(topic) == "tiltdrop/releasedropmotor" && message == "open")
    {
        if (manualMode)
        {
            // Servo blocking code—NOTE: This should be non-blocking in a final version!
            for (int pos = 90; pos >= 0; pos--)
                releaseServo.write(pos);
            releasedropMotorState = true;
        }
    }
    else if (String(topic) == "tiltdrop/releasedropmotor" && message == "close")
    {
        if (manualMode)
        {
            // Servo blocking code—NOTE: This should be non-blocking in a final version!
            for (int pos = 0; pos <= 90; pos++)
                releaseServo.write(pos);
            releasedropMotorState = false;
        }
    }
    else if (String(topic) == "rollercoaster/dispatch" && message == "go" && !manualMode)
    {
        coasterDispatched = true;
        Serial.println("recieved dipsatch go command");
    }

    if (String(topic) == "rollercoaster/event")
    {
        if (message == "train_on_lifthill")
        {
            tiltdropReady = true; // <-- zet flag
        }
    }
}

void connectMQTT()
{
    // Gebruik de clientId die eerder is gedefinieerd
    clientId = "roller-" + String(deviceName) + "-" + String((uint32_t)ESP.getEfuseMac());

    while (!client.connected())
    {
        Serial.print("Connecting MQTT...");
        // client.connect() zonder LWT parameters
        if (client.connect(clientId.c_str()))
        {
            Serial.println("connected");

            // Publiceer de initiële Heartbeat status (retained)
            client.publish(connectivityTopic, "online", true);

            // Subscribe naar control topics
            client.subscribe("tiltdrop/manual");
            client.subscribe("tiltdrop/tiltdropmotor");
            client.subscribe("tiltdrop/releasedropmotor");
            client.subscribe("rollercoaster/control/auto");
            client.subscribe("rollercoaster/event");
            client.subscribe("rollercoaster/dispatch");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.println(client.state());
            delay(2000);
        }
    }
}

void publishHeartbeat()
{
    if (millis() - lastHeartbeat >= heartbeatInterval)
    {
        lastHeartbeat = millis();
        client.publish(connectivityTopic, "online", true);
    }
}

// === Publish Status via MQTT (Onveranderd) ===
String lastStatus = "";

void publishStatusIfChanged()
{
    String status = "{";
    status += "\"hallSensorOnTiltdrop\":" + String(hallSensorOnTiltdropState ? "true" : "false");
    status += ",\"hallSensorTiltdropClosed\":" + String(hallSensorTiltdropClosedState ? "true" : "false");
    status += ",\"hallSensorTiltdropOpen\":" + String(hallSensorTiltdropOpenState ? "true" : "false");
    status += ",\"hallSensorOffTiltdrop\":" + String(hallSensorOffTiltdropState ? "true" : "false");
    status += ",\"tiltdropMotorMoving\":" + String(tiltdropMotorMoving ? "true" : "false");
    status += ",\"isTiltdropTrackOpen\":" + String(isTiltdropTrackOpen ? "true" : "false"); // Deze is nu veel accurater
    status += ",\"manualMode\":" + String(manualMode ? "true" : "false");
    status += ",\"releasedropMotorState\":" + String(releasedropMotorState ? "true" : "false");
    status += ",\"currentState\":\"" + currentStateName + "\"";
    status += "}";

    if (status != lastStatus)
    {
        Serial.println("[STATUS JSON] " + status);
        client.publish("tiltdrop/status", status.c_str(), true);
        lastStatus = status;
    }
}

void moveTiltdropMotor()
{
    if (tiltdropMotorMoving)
    {
        bool stillRunning = tiltTrackStepper.run();
        bool targetSensorHit = (tiltdropMotorState == true) ? hallSensorTiltdropOpenState : hallSensorTiltdropClosedState;

        if (motorRunState == SEEKING)
        {
            if (targetSensorHit)
            {
                motorRunState = OVERRUN;
                long overrunSteps = (tiltdropMotorState == true) ? -STEPS_OVERRUN : STEPS_OVERRUN;
                tiltTrackStepper.setCurrentPosition(0);
                tiltTrackStepper.move(overrunSteps);
                tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_SLOW);
            }
            else if (!motorInSlowZone)
            {
                long stepsMoved = abs(tiltTrackStepper.currentPosition() - motorReferencePosition);
                if (stepsMoved > STEPS_UNTIL_SLOWDOWN)
                {
                    motorInSlowZone = true;
                    tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_SLOW);
                }
            }
            else if (!stillRunning)
            {
                tiltdropMotorMoving = false;
                motorRunState = IDLE;
            }
        }
        else if (motorRunState == OVERRUN)
        {
            if (!stillRunning)
            {
                tiltdropMotorMoving = false;
                motorRunState = IDLE;
                isTiltdropTrackOpen = tiltdropMotorState;
            }
        }
    }
}

void handleAutoControl()
{
    static bool stateEntry = true;
    static unsigned long servoActionStart = 0;

    switch (currentState)
    {
    case STATE_IDLE:
        if (stateEntry)
        {
            Serial.println("[STATE] IDLE");
            stateEntry = false;
        }
        if (tiltdropReady && hallSensorOnTiltdropState)
        {
            client.publish("rollercoaster/event", "train_on_tiltdrop");
            tiltdropReady = false; // Trein is gearriveerd, Tower animatie stopt hierna ook
            coasterDispatched = false;
            setState(STATE_ON_TILTDROP);
        }
        break;

    case STATE_ON_TILTDROP:
        if (stateEntry)
        {
            client.publish("rollercoaster/event", "tiltdrop_opening");
            tiltTrackStepper.move(-9999999);
            tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_FAST);
            tiltdropMotorState = true;
            tiltdropMotorMoving = true;
            motorRunState = SEEKING;
            motorReferencePosition = tiltTrackStepper.currentPosition();
            motorInSlowZone = false;
            stateEntry = false;
        }
        moveTiltdropMotor();
        if (!tiltdropMotorMoving && hallSensorTiltdropOpenState)
        {
            client.publish("rollercoaster/event", "tiltdrop_open");
            isTiltdropTrackOpen = true;
            setState(STATE_TILTDROP_OPEN);
        }
        break;

    case STATE_TILTDROP_OPEN:
        if (stateEntry)
        {
            client.publish("rollercoaster/event", "releasedrop_opening");
            // Servo blocking (tijdelijk)
            for (int pos = 90; pos >= 0; pos--)
                releaseServo.write(pos);
            releasedropMotorState = true;
            ledDropControlState = true;
            servoActionStart = millis();
            stateEntry = false;
        }
        if (millis() - servoActionStart > 1000)
        {
            client.publish("rollercoaster/event", "releasedrop_open");
            setState(STATE_RESET_TILTDROP);
        }
        break;

    case STATE_RESET_TILTDROP:
        if (stateEntry)
        {
            client.publish("rollercoaster/event", "tiltdrop_resetting");
            for (int pos = 0; pos <= 90; pos++)
                releaseServo.write(pos);
            releasedropMotorState = false;

            tiltTrackStepper.move(9999999);
            tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_FAST);
            tiltdropMotorState = false;
            tiltdropMotorMoving = true;
            motorRunState = SEEKING;
            motorReferencePosition = tiltTrackStepper.currentPosition();
            motorInSlowZone = false;
            stateEntry = false;
            ledDropControlState = false;
        }
        moveTiltdropMotor();
        if (!tiltdropMotorMoving && hallSensorTiltdropClosedState)
        {
            client.publish("rollercoaster/event", "tiltdrop_closed");
            isTiltdropTrackOpen = false;
            setState(STATE_IDLE);
        }
        break;

    default:
        setState(STATE_IDLE);
        break;
    }

    static CoasterState prevState = STATE_ERROR;
    if (currentState != prevState)
    {
        prevState = currentState;
        stateEntry = true;
    }
}

void setup()
{
    Serial.begin(115200);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(300);
        Serial.print(".");
    }
    Serial.println("Connected!");

    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    connectMQTT();

    // === LED SETUP ===
    // Voeg alle strips toe aan FastLED
    FastLED.addLeds<NEOPIXEL, LEDS_TILT_PIN>(tiltLeds, NUM_LEDS_TILT);
    FastLED.addLeds<NEOPIXEL, LEDS_DROP_PIN>(dropLeds, NUM_LEDS_DROP);
    FastLED.addLeds<NEOPIXEL, LEDS_TOWER_PIN>(towerLeds, NUM_LEDS_TOWER);
    FastLED.setBrightness(255); // Globale max brightness

    pinMode(hallSensorOnTiltdrop, INPUT_PULLUP);
    pinMode(hallSensorTiltdropClosed, INPUT_PULLUP);
    pinMode(hallSensorTiltdropOpen, INPUT_PULLUP);
    pinMode(hallSensorOffTiltdrop, INPUT_PULLUP);

    tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_FAST);
    tiltTrackStepper.setAcceleration(200.0);
    releaseServo.attach(SERVO_PIN);

    updateTiltdropSensors();
    isTiltdropTrackOpen = hallSensorTiltdropOpenState;
    setState(STATE_IDLE);
}

void loop()
{
    if (!client.connected())
        connectMQTT();
    client.loop();

    updateTiltdropSensors();
    publishHeartbeat();
    publishStatusIfChanged();
    moveTiltdropMotor();

    // De nieuwe UpdateLeds functie roept alle effecten aan
    UpdateLeds();

    if (!manualMode)
        handleAutoControl();
}
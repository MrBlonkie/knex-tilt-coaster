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
const char *deviceName = "tiltdrop";
String clientId;
// De topic voor de connectiviteitsstatus (Heartbeat)
const char *connectivityTopic = "rollercoaster/tiltdrop/status";

// === Heartbeat Config (NON-BLOCKING) ===
unsigned long lastHeartbeat = 0;
// Heartbeat elke 2,5 seconden (voor 4,5s detectie)
const long heartbeatInterval = 2500;

// === Tilt-track motor config ===
#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23
AccelStepper tiltTrackStepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

// Drempels (deze moet je in de praktijk afstellen)
#define STEPS_UNTIL_SLOWDOWN 400 // Na hoeveel stappen vanaf start gaan we vertragen? (Totaal = 550)
#define STEPS_OVERRUN 40         // Hoeveel stappen extra na raken van sensor?

// Snelheden
#define MOTOR_SPEED_FAST 500.0
#define MOTOR_SPEED_SLOW 100.0

// State machine voor de motor
enum MotorRunState
{
    IDLE,
    SEEKING,
    OVERRUN
};
MotorRunState motorRunState = IDLE;
long motorReferencePosition = 0; // Om stappen te tellen voor slowdown
bool motorInSlowZone = false;    // Om te onthouden dat we al vertraagd zijn

bool tiltdropMotorState = false; // true = richting 'open', false = richting 'dicht'
bool tiltdropMotorMoving = false;
bool isTiltdropTrackOpen = false; // Wordt nu correct bijgewerkt door sensoren
bool coasterDispatched= false;

// === Servo config ===
#define SERVO_PIN 25
Servo releaseServo;
bool releasedropMotorState = false;

// === Sensors (Onveranderd) ===
#define hallSensorOnTiltdrop 34
#define hallSensorTiltdropClosed 26
#define hallSensorTiltdropOpen 32
#define hallSensorOffTiltdrop 27

bool hallSensorOnTiltdropState = false;
bool hallSensorTiltdropClosedState = false;
bool hallSensorTiltdropOpenState = false;
bool hallSensorOffTiltdropState = false;

bool tiltdropReady = false;
bool trainOnLifthill = true;
bool autoMode = false;

// === STATES (Onveranderd) ===
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
    Serial.println("[STATE] → " + currentStateName);
}

// === State control vars (Onveranderd) ===
bool manualMode = false;
bool tiltdropMotorManual = false;

// === Sensors update (Onveranderd) ===
void updateTiltdropSensors()
{
    bool onTiltdrop = digitalRead(hallSensorOnTiltdrop) == LOW;
    bool tiltdropClosed = digitalRead(hallSensorTiltdropClosed) == LOW;
    bool tiltdropOpen = digitalRead(hallSensorTiltdropOpen) == LOW;
    bool offTiltdrop = digitalRead(hallSensorOffTiltdrop) == LOW;

    // Update de globale state variabelen
    hallSensorOnTiltdropState = onTiltdrop;
    hallSensorTiltdropClosedState = tiltdropClosed;
    hallSensorTiltdropOpenState = tiltdropOpen;
    hallSensorOffTiltdropState = offTiltdrop;
}

// === Helper motor stop ===
void StopStepperMotor(AccelStepper &motor)
{
    motor.stop();
    motor.setCurrentPosition(motor.currentPosition()); // Reset 'huidige' positie
    motor.moveTo(motor.currentPosition());             // Vertel de motor om naar de huidige positie te gaan (dus: stop)
}

// === MQTT callback (AANGEPAST) ===
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
            tiltdropMotorState = false; // false = we zijn aan het sluiten
            tiltdropMotorMoving = true;
            tiltdropMotorManual = true;
            // isTiltdropTrackOpen = false; // NIET MEER HIER
            Serial.println("[MANUAL] Start motor: SLUITEN");
        }
    }
    // --- EINDE AANPASSING ---
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

// === MQTT connect ZONDER LWT ===
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

// === Publish Heartbeat via MQTT (NON-BLOCKING) ===
void publishHeartbeat()
{
    // Controleer of de intervaltijd verstreken is sinds de laatste Heartbeat
    if (millis() - lastHeartbeat >= heartbeatInterval)
    {
        lastHeartbeat = millis();

        // Heartbeat: stuur "online" status met Retain naar het specifieke topic.
        client.publish(connectivityTopic, "online", true);
        // Serial.println("[HB] Heartbeat sent: online"); // Uncomment voor debug
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
        // 1. Voer altijd een stap uit (indien nodig)
        bool stillRunning = tiltTrackStepper.run();

        // 2. Bepaal welke sensor we zoeken
        bool targetSensorHit = (tiltdropMotorState == true) ? hallSensorTiltdropOpenState : hallSensorTiltdropClosedState;

        // === STATE: SEEKING (Op zoek naar sensor) ===
        if (motorRunState == SEEKING)
        {
            if (targetSensorHit)
            {
                // Sensor geraakt! Start de 'OVERRUN'
                Serial.println("[MOTOR] Sensor hit. Starting OVERRUN.");
                motorRunState = OVERRUN;

                // Bepaal de extra stappen
                long overrunSteps = (tiltdropMotorState == true) ? -STEPS_OVERRUN : STEPS_OVERRUN;

                // Belangrijk: stel een *nieuw* relatief doel in
                // moveTo() zou het oude '9999999'-doel overschrijven, maar
                // move() telt op bij het huidige doel, wat we ook niet willen.
                // We stellen de huidige positie en gaan vanaf daar 'move'.
                tiltTrackStepper.setCurrentPosition(0); // Reset positie-teller
                tiltTrackStepper.move(overrunSteps);    // Stel nieuw, klein doel in

                tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_SLOW); // Overrun altijd langzaam
            }
            else if (!motorInSlowZone)
            {
                // We zijn nog 'fast' en de sensor is nog niet geraakt.
                // Controleren of we moeten vertragen?
                long stepsMoved = abs(tiltTrackStepper.currentPosition() - motorReferencePosition);
                if (stepsMoved > STEPS_UNTIL_SLOWDOWN)
                {
                    motorInSlowZone = true;
                    tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_SLOW);
                    Serial.println("[MOTOR] Entering slow zone.");
                }
            }
            else if (!stillRunning)
            {
                // Failsafe: De motor is gestopt omdat hij zijn (enorme) doel
                // heeft bereikt, wat niet zou mogen gebeuren.
                tiltdropMotorMoving = false;
                motorRunState = IDLE;
                Serial.println("[MOTOR] Tilt motor gestopt (FAILSAFE: stappenlimiet bereikt).");
            }
        }
        // === STATE: OVERRUN (Bezig met laatste stappen) ===
        else if (motorRunState == OVERRUN)
        {
            if (!stillRunning)
            {
                // De 'overrun' stappen zijn voltooid. Nu echt stoppen.
                Serial.println("[MOTOR] Overrun complete. Final position reached.");
                tiltdropMotorMoving = false;
                motorRunState = IDLE;

                // Update de definitieve status
                isTiltdropTrackOpen = tiltdropMotorState; // true als we openden, false als we sloten
            }
            // else: doe niets, laat 'run()' zijn werk doen.
        }
    }
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

void handleAutoControl() {
    static bool stateEntry = true; // wordt true bij elke nieuwe state
    static unsigned long servoActionStart = 0;

    switch (currentState) {

        // ------------------- STATE_IDLE -------------------
        case STATE_IDLE:
            if (stateEntry) {
                Serial.println("[STATE] IDLE");
                stateEntry = false;
            }

            // Wacht op signaal van station (train_on_lifthill)
            if (tiltdropReady && hallSensorOnTiltdropState) {
                client.publish("rollercoaster/event", "train_on_tiltdrop");
                tiltdropReady = false; // reset flag, enkel 1x gebruiken
                coasterDispatched = false;
                setState(STATE_ON_TILTDROP);
            }
            break;


        // ------------------- STATE_ON_TILTDROP -------------------
        case STATE_ON_TILTDROP:
            if (stateEntry) {
                Serial.println("[STATE] ON_TILTDROP → start kantelen");
                client.publish("rollercoaster/event", "tiltdrop_opening");

                // Start de tiltbeweging naar "open"
                tiltTrackStepper.move(-9999999);
                tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_FAST);
                tiltdropMotorState = true;   // richting open
                tiltdropMotorMoving = true;
                motorRunState = SEEKING;
                motorReferencePosition = tiltTrackStepper.currentPosition();
                motorInSlowZone = false;

                stateEntry = false;
            }

            moveTiltdropMotor();

            // Zodra sensor geraakt
            if (!tiltdropMotorMoving && hallSensorTiltdropOpenState) {
                client.publish("rollercoaster/event", "tiltdrop_open");
                isTiltdropTrackOpen = true;
                setState(STATE_TILTDROP_OPEN);
            }
            break;


        // ------------------- STATE_TILTDROP_OPEN -------------------
        case STATE_TILTDROP_OPEN:
            if (stateEntry) {
                Serial.println("[STATE] TILTDROP_OPEN → open release servo");
                client.publish("rollercoaster/event", "releasedrop_opening");

                // Start servo beweging (tijdelijk blocking → later non-blocking maken)
                for (int pos = 90; pos >= 0; pos--) releaseServo.write(pos);
                releasedropMotorState = true;

                servoActionStart = millis();
                stateEntry = false;
            }

            // korte wachttijd om servo fysiek te laten bewegen
            if (millis() - servoActionStart > 1000) {
                client.publish("rollercoaster/event", "releasedrop_open");
                setState(STATE_RESET_TILTDROP);
            }
            break;


        // ------------------- STATE_RESET_TILTDROP -------------------
        case STATE_RESET_TILTDROP:
            if (stateEntry) {
                Serial.println("[STATE] RESET_TILTDROP → terug kantelen");
                client.publish("rollercoaster/event", "tiltdrop_resetting");

                // Sluit release servo
                for (int pos = 0; pos <= 90; pos++) releaseServo.write(pos);
                releasedropMotorState = false;

                // Start motor richting dicht
                tiltTrackStepper.move(9999999);
                tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_FAST);
                tiltdropMotorState = false; // richting dicht
                tiltdropMotorMoving = true;
                motorRunState = SEEKING;
                motorReferencePosition = tiltTrackStepper.currentPosition();
                motorInSlowZone = false;

                stateEntry = false;
            }

            moveTiltdropMotor();

            if (!tiltdropMotorMoving && hallSensorTiltdropClosedState) {
                client.publish("rollercoaster/event", "tiltdrop_closed");
                isTiltdropTrackOpen = false;
                setState(STATE_IDLE);
            }
            break;


        // ------------------- DEFAULT (failsafe) -------------------
        default:
            setState(STATE_IDLE);
            break;
    }

    // Reset entry flag bij state change
    static CoasterState prevState = STATE_ERROR;
    if (currentState != prevState) {
        prevState = currentState;
        stateEntry = true;
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

    // De eerste connectie wordt nu hier gedaan (zonder LWT), en de Heartbeat wordt direct verstuurd
    connectMQTT();

    // LEDs
    FastLED.addLeds<NEOPIXEL, LEDS_TILT_PIN>(tiltLeds, NUM_LEDS_TILT);

    // Sensors
    pinMode(hallSensorOnTiltdrop, INPUT_PULLUP);
    pinMode(hallSensorTiltdropClosed, INPUT_PULLUP);
    pinMode(hallSensorTiltdropOpen, INPUT_PULLUP);
    pinMode(hallSensorOffTiltdrop, INPUT_PULLUP);

    // Motor
    tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_FAST); // Gebruik de nieuwe define
    tiltTrackStepper.setAcceleration(200.0);

    // Servo
    releaseServo.attach(SERVO_PIN);

    // Bepaal de initiële staat van de track
    updateTiltdropSensors();
    if (hallSensorTiltdropOpenState)
    {
        isTiltdropTrackOpen = true;
    }
    else if (hallSensorTiltdropClosedState)
    {
        isTiltdropTrackOpen = false;
    }
    else
    {
        // Onbekende staat, ga uit van dicht
        isTiltdropTrackOpen = false;
        Serial.println("[INIT] Waarschuwing: Tiltdrop positie onbekend, ga uit van 'dicht'.");
    }

    setState(STATE_IDLE);
}

// === Loop ===
void loop()
{
    if (!client.connected())
        connectMQTT();
    client.loop();

    updateTiltdropSensors();

    // NON-BLOCKING Heartbeat toegevoegd
    publishHeartbeat();

    moveTiltdropMotor();

    UpdateLeds();

    if (!manualMode)
    {
        handleAutoControl();
    }

    // Status publish
    publishStatusIfChanged();
}
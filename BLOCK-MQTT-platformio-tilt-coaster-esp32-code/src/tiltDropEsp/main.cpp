#include <WiFi.h>
#include <PubSubClient.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <FastLED.h>
#include "LedSetup.h"
#include "../shared/env.h"

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

bool tiltdropMotorState = false; // False = Dicht (Boven), True = Open (Beneden)
bool tiltdropMotorMoving = false;
bool isTiltdropTrackOpen = false;
bool coasterDispatched = false;

// === Servo config ===
#define SERVO_PIN 25
Servo releaseServo;
bool releasedropMotorState = false;
int currentPos = 90;
int targetPos = 90; // Default Dicht
unsigned long lastStep = 0;
const unsigned long stepInterval = 2;

// Non-blocking servo sweep
bool moveServoSmooth(Servo &servo, int &current, int target)
{
    unsigned long now = millis();
    if (now - lastStep < stepInterval)
        return false;
    lastStep = now;

    if (current < target)
    {
        current++;
        servo.write(current);
        return (current == target);
    }
    else if (current > target)
    {
        current--;
        servo.write(current);
        return (current == target);
    }
    return true;
}

// === Sensors ===
#define hallSensorOnTiltdrop 34
#define hallSensorTiltdropClosed 26
#define hallSensorTiltdropOpen 32
#define hallSensorOffTiltdrop 27

bool hallSensorOnTiltdropState = false;
bool hallSensorTiltdropClosedState = false;
bool hallSensorTiltdropOpenState = false;
bool hallSensorOffTiltdropState = false;

// === Mist Effect ===
#define mistEffectRelay 5
bool mistEffectState = false;

// === Logic Flags ===
bool cartOnTiltdrop = false;
bool ledDropControlState = false;
bool trainOnLifthill = false;
bool trainOnLayout = false;

bool isTiltdropOccupied = false;
bool isNextBlockFree = false; // Let op: Start op FALSE (veiligheid), wacht op MQTT bericht
bool isTiltdropResetted = false;

bool trainOnTiltdropFlag = false;
bool tiltdropOpeningFlag = false;
bool tiltdropSafetyFlag = false;
bool isTiltdropFreeFlag = false;

String latestEventMsg = "";
bool manualMode = false;

// timers
unsigned long releaseTimer = 0;
unsigned long resetTimer = 0;
const int delayBeforeRelease = 2000; // 2 seconden wachten voor de drop
const int delayBeforeReset = 1000;   // 1 seconde wachten voor de tilt teruggaat

// === HELPER FUNCTIES (Trigger Motoren) ===
// Deze vervangen jouw 'handleTiltMotorControl' loop-fout.
// Ze worden maar 1x aangeroepen als de actie moet starten.

void StartTiltingDown()
{
    if (tiltdropMotorMoving && tiltdropMotorState == true)
        return; // Al bezig
    if (hallSensorTiltdropOpenState)
        return; // Al open

    Serial.println("ACTION: Start Opening (Down)");
    tiltTrackStepper.move(-9999999);
    tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_FAST);

    tiltdropMotorState = true; // Doel = Open
    tiltdropMotorMoving = true;

    motorReferencePosition = tiltTrackStepper.currentPosition();
    motorRunState = SEEKING;
    motorInSlowZone = false;
}

void StartTiltingUp()
{
    if (tiltdropMotorMoving && tiltdropMotorState == false)
        return; // Al bezig
    if (hallSensorTiltdropClosedState)
        return; // Al dicht

    Serial.println("ACTION: Start Closing (Up)");
    tiltTrackStepper.move(9999999);
    tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_FAST);

    tiltdropMotorState = false; // Doel = Dicht
    tiltdropMotorMoving = true;

    motorReferencePosition = tiltTrackStepper.currentPosition();
    motorRunState = SEEKING;
    motorInSlowZone = false;
}

// === Led Control
void UpdateLeds()
{
    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate < 20)
        return; // 50 Hz max
    lastLedUpdate = millis();
    static bool whiteFlashesDone = false;

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

    UpdateDropLeds(ledDropControlState);
    UpdateTowerLeds(trainOnLifthill);
    FastLED.show();
}

// === MQTT callback ===
void callback(char *topic, byte *payload, unsigned int length)
{
    String message = "";
    for (int i = 0; i < length; i++)
        message += (char)payload[i];

    if (String(topic) == "tiltdrop/manual")
    {
        if (message == "on")
            manualMode = true;
        else if (message == "off")
        {
            manualMode = false;
            coasterDispatched = false;
            StartTiltingUp(); // Veiligheid: sluiten bij auto mode
            targetPos = 90;
        }
    }

    // Alleen luisteren naar motor commando's in manual mode
    if (manualMode)
    {
        if (String(topic) == "tiltdrop/tiltdropmotor")
        {
            if (message == "open")
                StartTiltingDown();
            if (message == "close")
                StartTiltingUp();
        }
        if (String(topic) == "tiltdrop/releasedropmotor")
        {
            if (message == "open")
            {
                targetPos = 0;
                releasedropMotorState = true;
            }
            if (message == "close")
            {
                targetPos = 90;
                releasedropMotorState = false;
            }
        }
        if (String(topic) == "tiltdrop/misteffect")
        {
            if (message == "on")
            {
                digitalWrite(mistEffectRelay, LOW);
                mistEffectState = true;
            }
            if (message == "off")
            {
                digitalWrite(mistEffectRelay, HIGH);
                mistEffectState = false;
            }
        }
    }

    if (String(topic) == "rollercoaster/dispatch" && message == "go" && !manualMode)
    {
        coasterDispatched = true;
        Serial.println("recieved dispatch go command");
    }
    if (String(topic) == "rollercoaster/dispatch" && message == "stop")
    {
        coasterDispatched = false;
    }

    if (String(topic) == "rollercoaster/clear/tiltdrop" && message == "clear")
    {
        tiltdropSafetyFlag = false;
        client.publish("rollercoaster/event", "cleared_tiltdrop_safety_flag");
    }

    // Events van andere ESPs
    if (String(topic) == "rollercoaster/event")
    {
        if (message == "train_on_lifthill")
            trainOnLifthill = true;
    }

    // Block System Events
    if (String(topic) == "rollercoaster/block/event")
    {
        if (message == "layout_free")
            isNextBlockFree = true;
        if (message == "layout_occupied")
            isNextBlockFree = false;
    }

    if (String(topic) == "rollercoaster/estop" && message == "stop")
    {
        tiltTrackStepper.stop();
        motorRunState = IDLE;
        tiltdropMotorMoving = false;
        coasterDispatched = false;
        targetPos = 90;
        releasedropMotorState = false;
        client.publish("rollercoaster/event", "estop_tiltdrop");
    }
}

// === Setup & Connect (Standaard) ===
void connectMQTT()
{
    clientId = "roller-" + String(deviceName) + "-" + String((uint32_t)ESP.getEfuseMac());
    while (!client.connected())
    {
        Serial.print("Connecting MQTT...");
        if (client.connect(clientId.c_str()))
        {
            Serial.println("connected");
            client.publish(connectivityTopic, "online", true);

            client.subscribe("tiltdrop/manual");
            client.subscribe("tiltdrop/tiltdropmotor");
            client.subscribe("tiltdrop/releasedropmotor");
            client.subscribe("tiltdrop/misteffect");
            client.subscribe("rollercoaster/event");
            client.subscribe("rollercoaster/dispatch");

            client.subscribe("rollercoaster/block/event"); // Belangrijk!

            client.subscribe("rollercoaster/estop");
            client.subscribe("rollercoaster/reset");
            client.subscribe("rollercoaster/clear/tiltdrop");
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

// === Publish Status via MQTT ===
String lastStatus = "";

void publishStatusIfChanged()
{
    String status = "{";

    status += "\"sensors\":{";
    status += "\"hallSensorOnTiltdropState\":" + String(hallSensorOnTiltdropState ? "true" : "false");
    status += ",\"hallSensorTiltdropClosedState\":" + String(hallSensorTiltdropClosedState ? "true" : "false");
    status += ",\"hallSensorTiltdropOpenState\":" + String(hallSensorTiltdropOpenState ? "true" : "false");
    status += ",\"hallSensorOffTiltdropState\":" + String(hallSensorOffTiltdropState ? "true" : "false");
    status += "},";

    status += "\"train\":{";
    status += "\"cartOnTiltdrop\":" + String(cartOnTiltdrop ? "true" : "false");
    status += ",\"trainOnLayout\":" + String(trainOnLayout ? "true" : "false");
    status += "},";

    status += "\"tiltdrop\":{";
    status += "\"tiltdropMotorMoving\":" + String(tiltdropMotorMoving ? "true" : "false");
    status += ",\"isTiltdropTrackOpen\":" + String(isTiltdropTrackOpen ? "true" : "false");
    status += ",\"releasedropMotorState\":" + String(releasedropMotorState ? "true" : "false");
    status += ",\"misteffectState\":" + String(mistEffectState ? "true" : "false");
    status += "},";

    status += "\"mode\":{";
    status += "\"manualMode\":" + String(manualMode ? "true" : "false");
    status += "},";

    status += "\"blocks\":{";
    status += "\"isTiltdropOccupied\":" + String(isTiltdropOccupied ? "true" : "false");
    status += ",\"isNextBlockFree\":" + String(isNextBlockFree ? "true" : "false");
    status += "},";

    status += "\"coaster\":{";
    status += "\"coasterDispatched\":" + String(coasterDispatched ? "true" : "false");
    status += "},";

    status += "\"flags\":{";
    status += "\"trainOnTiltdropFlag\":" + String(trainOnTiltdropFlag ? "true" : "false");
    status += ",\"tiltdropOpeningFlag\":" + String(tiltdropOpeningFlag ? "true" : "false");
    status += ",\"tiltdropSafetyFlag\":" + String(tiltdropSafetyFlag ? "true" : "false");
    status += ",\"isTiltdropFreeFlag\":" + String(isTiltdropFreeFlag ? "true" : "false");
    status += "}";

    status += "}";

    if (status != lastStatus)
    {
        Serial.println("[STATUS JSON] " + status);
        client.publish("tiltdrop/status", status.c_str(), true);
        lastStatus = status;
    }
}

void updateTiltdropSensors()
{
    hallSensorOnTiltdropState = (digitalRead(hallSensorOnTiltdrop) == LOW);
    hallSensorTiltdropClosedState = (digitalRead(hallSensorTiltdropClosed) == LOW);
    hallSensorTiltdropOpenState = (digitalRead(hallSensorTiltdropOpen) == LOW);
    hallSensorOffTiltdropState = (digitalRead(hallSensorOffTiltdrop) == LOW);
}

void handleTiltdropBlockV2()
{
    // enter tiltdrop logic
    if (hallSensorOnTiltdropState && !trainOnTiltdropFlag)
    {
        isTiltdropOccupied = true;
        trainOnTiltdropFlag = true;
        isTiltdropResetted = false;
        trainOnLifthill = false;
        client.publish("rollercoaster/event", "train_on_tiltdrop");
        client.publish("rollercoaster/block/event", "tiltdrop_occupied");
    }

    // open tiltdrop logic
    if (isTiltdropOccupied && hallSensorTiltdropClosedState && !tiltdropOpeningFlag && !tiltdropMotorMoving && !tiltdropSafetyFlag)
    {
        StartTiltingDown();
        client.publish("rollercoaster/event", "tiltdrop_opening");
        tiltdropOpeningFlag = true;
    }

    // release logic
    if (isTiltdropOccupied && hallSensorTiltdropOpenState && !tiltdropMotorMoving && isNextBlockFree && !tiltdropSafetyFlag)
    {
        if (releaseTimer == 0)
        {
            releaseTimer = millis();
            Serial.println("Release timer gestart...");
            digitalWrite(mistEffectRelay, LOW);
            mistEffectState = true;
        }

        if (millis() - releaseTimer >= delayBeforeRelease && !releasedropMotorState)
        {
            targetPos = 0;
            releasedropMotorState = true;
            client.publish("rollercoaster/event", "releasedrop_opening");
            digitalWrite(mistEffectRelay, HIGH);
            mistEffectState = false;
        }
    }

    // reset logic
    if (isTiltdropOccupied && releasedropMotorState && hallSensorOffTiltdropState && !tiltdropSafetyFlag)
    {
            client.publish("rollercoaster/event", "resetting_tiltdrop");
            client.publish("rollercoaster/block/event", "layout_occupied");
            targetPos = 90; // Servo DICHT
            releasedropMotorState = false;
            releaseTimer = 0;
            StartTiltingUp();
            trainOnTiltdropFlag = false;
    }

    // give clear tiltdrop
    if (hallSensorTiltdropClosedState && !isTiltdropResetted && !tiltdropMotorMoving && !releasedropMotorState)
    {
        isTiltdropResetted = true;
        isTiltdropOccupied = false;
        tiltdropOpeningFlag = false;

        client.publish("rollercoaster/block/event", "tiltdrop_free");
        client.publish("rollercoaster/event", "tiltdrop_closed_and_safe");
    }

    // safety
    if (isTiltdropOccupied)
    {
        if (releasedropMotorState && !hallSensorTiltdropOpenState)
        {
            tiltdropSafetyFlag = true;
            client.publish("rollercoaster/event", "CRITICAL_ERROR: Tiltdrop release triggered while not level!");
        }
    }

    // tiltdrop free on start
    if (hallSensorTiltdropClosedState && !releasedropMotorState && !isTiltdropOccupied && !isTiltdropFreeFlag)
    {
        isTiltdropFreeFlag = true;
        client.publish("rollercoaster/block/event", "tiltdrop_free");
    }
}

// === MOTOR PHYSICS (De Loop) ===
void moveTiltdropMotor()
{
    if (tiltdropMotorMoving)
    {
        bool stillRunning = tiltTrackStepper.run();

        // Bepaal welke sensor het doel is
        bool targetSensorHit = (tiltdropMotorState == true) ? hallSensorTiltdropOpenState : hallSensorTiltdropClosedState;

        if (motorRunState == SEEKING)
        {
            if (targetSensorHit)
            {
                // Sensor geraakt -> Overrun fase
                motorRunState = OVERRUN;
                long overrunSteps = (tiltdropMotorState == true) ? -STEPS_OVERRUN : STEPS_OVERRUN;
                tiltTrackStepper.setCurrentPosition(0);
                tiltTrackStepper.move(overrunSteps);
                tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_SLOW);
            }
            else if (!motorInSlowZone)
            {
                // Check of we al moeten afremmen op basis van stappen
                if (abs(tiltTrackStepper.currentPosition() - motorReferencePosition) > STEPS_UNTIL_SLOWDOWN)
                {
                    motorInSlowZone = true;
                    tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_SLOW);
                }
            }
            else if (!stillRunning)
            {
                // Failsafe: motor denkt dat hij klaar is zonder sensor
                tiltdropMotorMoving = false;
                motorRunState = IDLE;
            }
        }
        else if (motorRunState == OVERRUN)
        {
            if (!stillRunning)
            {
                // Klaar met beweging
                tiltdropMotorMoving = false;
                motorRunState = IDLE;

                // Status update
                if (tiltdropMotorState == true)
                    isTiltdropTrackOpen = true;
                else
                    isTiltdropTrackOpen = false;
            }
        }
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

    // LEDS
    FastLED.addLeds<NEOPIXEL, LEDS_TILT_PIN>(tiltLeds, NUM_LEDS_TILT);
    FastLED.addLeds<NEOPIXEL, LEDS_DROP_PIN>(dropLeds, NUM_LEDS_DROP);
    FastLED.addLeds<NEOPIXEL, LEDS_TOWER_PIN>(towerLeds, NUM_LEDS_TOWER);
    FastLED.setBrightness(255);

    // Pins
    pinMode(hallSensorOnTiltdrop, INPUT_PULLUP);
    pinMode(hallSensorTiltdropClosed, INPUT_PULLUP);
    pinMode(hallSensorTiltdropOpen, INPUT_PULLUP);
    pinMode(hallSensorOffTiltdrop, INPUT_PULLUP);
    
    pinMode(mistEffectRelay, OUTPUT);
    digitalWrite(mistEffectRelay, HIGH);

    // Motor
    tiltTrackStepper.setMaxSpeed(MOTOR_SPEED_FAST);
    tiltTrackStepper.setAcceleration(200.0);

    // Servo
    releaseServo.attach(SERVO_PIN);
    targetPos = 90;

    updateTiltdropSensors();
    isTiltdropTrackOpen = hallSensorTiltdropOpenState;
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
    moveServoSmooth(releaseServo, currentPos, targetPos);

    UpdateLeds();

    if (coasterDispatched)
    {
        handleTiltdropBlockV2();
    }
}
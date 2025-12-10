#include <AccelStepper.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include "../shared/env.h"

// === WiFi & MQTT ===
WiFiClient espClient;
PubSubClient client(espClient);
const char *deviceName = "switchtrack";
String clientId;
const char *connectivityTopic = "rollercoaster/switchtrack/status";

// === Heartbeat Config ===
unsigned long lastHeartbeat = 0;
const long heartbeatInterval = 2500;

// === Motor Config ===
#define ROTATING_IN1 19
#define ROTATING_IN2 21
#define ROTATING_IN3 22
#define ROTATING_IN4 23

#define TILTING_IN1 14
#define TILTING_IN2 13
#define TILTING_IN3 15
#define TILTING_IN4 18

// === Helper motor stop ===
void StopStepperMotor(AccelStepper &motor)
{
  motor.stop();
  motor.setCurrentPosition(motor.currentPosition());
  motor.moveTo(motor.currentPosition());
}

// === Servo config ===
#define SERVO_PIN 33
Servo releaseServo;
bool releaseswitchMotorState = false;

int currentPos = 90;
int targetPos = 0;
unsigned long lastServoStep = 0;
const unsigned long stepInterval = 2;

// === Status variabalen voor beweging ===
long lastTiltStep = 0;
// PAS OP: Als dit interval te klein is, gaat hij weer constant draaien.
// 1000 is een veilige startwaarde.
const long tiltInterval = 100;
const int tiltStepSize = -60; // Iets verhoogd (was -5) om het effect duidelijker te maken, pas aan indien nodig.

// Logica vlaggen
bool droppingToStation = false;
bool doingExtraSteps = false;

// Instelling: Hoeveel stappen EXTRA na het zien van de stationsensor?
const int extraTiltSteps = 50;

bool moveServoSmooth(Servo &servo, int &current, int target)
{
  static unsigned long lastStepLocal = 0;
  unsigned long now = millis();
  if (now - lastStepLocal < stepInterval)
    return false;
  lastStepLocal = now;

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

// === Sensor config ===
#define hallSensorStationConnect 27
#define hallSensorBrakeConnect 26
#define hallSensorMiddle 25
#define hallSensorOnSwitchtrack 32

AccelStepper rotatingStepper(AccelStepper::FULL4WIRE, ROTATING_IN1, ROTATING_IN3, ROTATING_IN2, ROTATING_IN4);
AccelStepper tiltingStepper(AccelStepper::FULL4WIRE, TILTING_IN1, TILTING_IN3, TILTING_IN2, TILTING_IN4);

bool hallSensorStationConnectState = false;
bool hallSensorMiddleState = false;
bool hallSensorBrakeConnectState = false;
bool hallSensorOnSwitchtrackState = false;

bool manualMode = false;
bool coasterDispatched = false;
String manualRotateTarget = "";
float targetSpeed = 0;
bool isSwitchtrackAtBrakes = false;

bool trainOnLayout = false;
String latestEventMsg = "";

// === Tilt instellingen per bestemming ===
const int tiltStepSizeBrakes = -50;    // richting brakes
const int tiltStepSizeStation = 12;    // richting station (reverse van brakes)
const int tiltExtraStepsBrakes = 100;  // extra stappen na brakes
const int tiltExtraStepsStation = 150; // extra stappen na station
const long tiltIntervalBrakes = 80;    // interval voor tilt-update
const long tiltIntervalStation = 110;  // interval voor tilt-update

// Flags voor fasebeheer
bool droppingToTarget = false;
bool doingExtraTiltSteps = false;
bool trainInStationFlag = false;

unsigned long stateStartTime = 0; // Timer voor in de state machine
bool timerActive = false;         // Om te checken of we aan het wachten zijn

// === STATES ===
enum CoasterState
{
  STATE_IDLE,
  STATE_WAIT_FOR_BRAKES,
  STATE_ENTER_SWITCHTRACK,
  STATE_ROTATE_SWITCHTRACK,
  STATE_RELEASE,
  STATE_RESET,
  STATE_ERROR
};
CoasterState currentState = STATE_IDLE;
String currentStateName = "IDLE";

void setState(CoasterState newState)
{
  currentState = newState;
  stateStartTime = millis(); // RESET DE TIMER BIJ ELKE STATE WISSEL
  timerActive = false;

  switch (newState)
  {
  case STATE_IDLE:
    currentStateName = "IDLE";
    break;

  case STATE_WAIT_FOR_BRAKES:
    currentStateName = "WAIT_FOR_BRAKES";
    break;

  case STATE_ENTER_SWITCHTRACK:
    currentStateName = "ENTER_SWITCHTRACK";
    break;

  case STATE_ROTATE_SWITCHTRACK:
    currentStateName = "ROTATE_SWITCHTRACK";
    break;

  case STATE_RELEASE:
    currentStateName = "RELEASE";
    break;

  case STATE_RESET:
    currentStateName = "RESET";
    break;

  case STATE_ERROR:
    currentStateName = "ERROR";
    break;

  default:
    currentStateName = "UNKNOWN";
    break;
  }

  Serial.println("[STATE] -> " + currentStateName);
}

void handleMovement()
{
  // AANPASSING: We kijken hier NIET meer naar manualMode.
  // Als targetSpeed 0 is, stoppen we. Anders bewegen we.
  if (targetSpeed == 0)
  {
    rotatingStepper.stop();
    StopStepperMotor(tiltingStepper);
    droppingToTarget = false;
    doingExtraTiltSteps = false;
    return;
  }

  // Tilt accumulator
  static long tiltAccumulator = 0;
  static long lastRotPos = 0;
  long rotDelta = abs(rotatingStepper.currentPosition() - lastRotPos);
  tiltAccumulator += rotDelta;
  lastRotPos = rotatingStepper.currentPosition();

  // === BEWEGING NAAR BRAKES (Negatieve snelheid) ===
  if (targetSpeed < 0)
  {
    droppingToTarget = false;
    doingExtraTiltSteps = false;

    // Sensor check: zijn we er al?
    if (hallSensorBrakeConnectState)
    {
      rotatingStepper.stop();
      tiltingStepper.stop();
      targetSpeed = 0; // Reset snelheid omdat doel bereikt is
      Serial.println("Brakes sensor active: motors stopped.");
      tiltAccumulator = 0;
      return;
    }

    rotatingStepper.setSpeed(targetSpeed);
    rotatingStepper.runSpeed();

    if (tiltAccumulator >= tiltIntervalBrakes)
    {
      tiltingStepper.move(tiltStepSizeBrakes);
      tiltAccumulator = 0;
    }
    tiltingStepper.run();
    return;
  }

  // === BEWEGING NAAR STATION (Positieve snelheid) ===
  if (targetSpeed > 0)
  {
    droppingToTarget = false;
    doingExtraTiltSteps = false;

    // Sensor check: zijn we er al?
    if (hallSensorMiddleState)
    { // Let op: In je oude code stond hier hallSensorMiddleState, klopt dat? Of moet dit hallSensorStationConnectState zijn?
      rotatingStepper.stop();
      tiltingStepper.stop();
      targetSpeed = 0; // Reset snelheid omdat doel bereikt is
      Serial.println("Station sensor active: motors stopped.");
      tiltAccumulator = 0;
      return;
    }

    rotatingStepper.setSpeed(targetSpeed);
    rotatingStepper.runSpeed();

    if (tiltAccumulator >= tiltIntervalStation)
    {
      tiltingStepper.move(tiltStepSizeStation);
      tiltAccumulator = 0;
    }
    tiltingStepper.run();
    return;
  }
}

// === MQTT callback ===
void callback(char *topic, byte *payload, unsigned int length)
{
  String message;
  for (int i = 0; i < length; i++)
    message += (char)payload[i];

  if (String(topic) == "switchtrack/manual")
  {
    if (message == "on")
    {
      manualMode = true;
      Serial.println("Manual mode ENABLED");
    }
    else if (message == "off")
    {
      manualMode = false;
      Serial.println("Manual mode DISABLED");
    }
  }
  else if (String(topic) == "rollercoaster/dispatch" && message == "go")
  {
    coasterDispatched = true;
    Serial.println("Received dispatch GO command");
  }
  else if (manualMode)
  {
    if (String(topic) == "switchtrack/rotatemotor")
    {
      if (message == "on")
      {
        Serial.println("COMMAND: Rotate to Brakes");
        targetSpeed = -500;
        manualRotateTarget = "brakes";
        lastTiltStep = rotatingStepper.currentPosition(); // Reset lastTiltStep zodat hij niet meteen een enorme sprong maakt als de motor lang stil stond
      }
      if (message == "off")
      {
        Serial.println("COMMAND: Rotate to Station");
        targetSpeed = 500;
        manualRotateTarget = "station";
      }
    }
    else if (String(topic) == "switchtrack/releaseswitchtrackmotor")
    {
      if (message == "open")
      {
        Serial.println("COMMAND: Servo Open");
        targetPos = 90;
        releaseswitchMotorState = true;
      }
      if (message == "close")
      {
        Serial.println("COMMAND: Servo Close");
        targetPos = 0;
        releaseswitchMotorState = false;
      }
    }
    else if (String(topic) == "rollercoaster/dispatch" && message == "go" && !manualMode)
    {
      coasterDispatched = true;
      Serial.println("recieved dipsatch go command");
    }
  }
  if (String(topic) == "rollercoaster/event" && message == "train_enters_station")
  {
    trainInStationFlag = true; // <-- zet flag
  }
}

// === Publish Heartbeat ===
void publishHeartbeat()
{
  if (millis() - lastHeartbeat >= heartbeatInterval)
  {
    lastHeartbeat = millis();
    if (client.connected())
    {
      Serial.println("HB publish");
      client.publish(connectivityTopic, "online", true);
    }
    else
    {
      Serial.println("[HB] MQTT not connected, skipping publish");
    }
  }
}

// === Sensors update ===
void updateSensors()
{
  hallSensorBrakeConnectState = digitalRead(hallSensorBrakeConnect) == LOW;
  hallSensorMiddleState = digitalRead(hallSensorMiddle) == LOW;
  hallSensorStationConnectState = digitalRead(hallSensorStationConnect) == LOW;
  hallSensorOnSwitchtrackState = digitalRead(hallSensorOnSwitchtrack) == LOW;
}

// === Publish Status ===
String lastStatus = "";
void publishStatusIfChanged()
{
  String status = "{";
  status += "\"hallSensorBrakeConnect\":" + String(hallSensorBrakeConnectState ? "true" : "false");
  status += ",\"hallSensorMiddle\":" + String(hallSensorMiddleState ? "true" : "false");
  status += ",\"hallSensorStationConnect\":" + String(hallSensorStationConnectState ? "true" : "false");
  status += ",\"hallSensorOnSwitchtrack\":" + String(hallSensorOnSwitchtrackState ? "true" : "false");
  status += ",\"releaseswitchMotorState\":" + String(releaseswitchMotorState ? "true" : "false");
  status += ",\"manualMode\":" + String(manualMode ? "true" : "false");
  status += ",\"currentState\":\"" + manualRotateTarget + "\"";
  status += "}";

  if (status != lastStatus)
  {
    client.publish("switchtrack/status", status.c_str(), true);
    lastStatus = status;
  }
}

// === Connect MQTT ===
void connectMQTT()
{
  clientId = "roller-" + String(deviceName) + "-" + String((uint32_t)ESP.getEfuseMac());

  while (!client.connected())
  {
    Serial.print("Connecting to MQTT...");
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");
      client.publish(connectivityTopic, "online", true);

      client.subscribe("switchtrack/manual");
      client.subscribe("switchtrack/rotatemotor");
      client.subscribe("switchtrack/releaseswitchtrackmotor");
      client.subscribe("rollercoaster/control/auto");
      client.subscribe("rollercoaster/event");
      client.subscribe("rollercoaster/dispatch");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(", retrying in 2s...");
      delay(2000);
    }
  }
}

// AUTO LOGIC
void handleAutoControl()
{

  switch (currentState)
  {
  case STATE_IDLE:
    if (coasterDispatched)
    {
      coasterDispatched = false;
      Serial.println("[AUTO][IDLE] Dispatch received -> WAIT_FOR_BRAKES");
      setState(STATE_WAIT_FOR_BRAKES);
    }
    break;

  case STATE_WAIT_FOR_BRAKES:
    // TODO
    // 
    // 
    setState(STATE_ENTER_SWITCHTRACK);
    break;

  case STATE_ENTER_SWITCHTRACK:
    if (hallSensorOnSwitchtrackState)
    {
      if (!timerActive)
      {
        stateStartTime = millis();
        timerActive = true;
        Serial.println("[AUTO] Train detected on switchtrack. 1s stabilization timer started.");
      }
      if (millis() - stateStartTime >= 2000)
      {
        targetSpeed = 500;
        manualRotateTarget = "station";
        Serial.println("[AUTO] Sensor active -> Rotating to station");
        client.publish("rollercoaster/event", "rotating_switchtrack_to_station");
        setState(STATE_RELEASE);
      }
    }
    break;

  case STATE_RELEASE:
    if (hallSensorStationConnectState)
    {
      targetPos = 90; // open servo
      releaseswitchMotorState = true;
      Serial.println("[AUTO] Train released -> RESET");
      client.publish("rollercoaster/event", "released_train_switchtrack");
      setState(STATE_RESET);
    }
    break;

  case STATE_RESET:
    if (trainInStationFlag)
    {
      if (!timerActive)
      {
        stateStartTime = millis();
        timerActive = true;
      }
      if (millis() - stateStartTime >= 1000)
      {
        targetPos = 0; // close servo
        releaseswitchMotorState = false;

        targetSpeed = -500; // stepper dir brakes
        manualRotateTarget = "brakes";

        trainInStationFlag = false;
      }
    }

    if (hallSensorBrakeConnectState && manualRotateTarget == "brakes")
    {
      Serial.println("[AUTO][RESET] Brake sensor reached -> IDLE");
      client.publish("rollercoaster/event", "resetted_switchtrack");
      targetSpeed = 0;
      setState(STATE_IDLE);
    }
    break;

  case STATE_ERROR:
    // Safety state
    break;
  }
}

void setup()
{
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(300);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setKeepAlive(30);
  connectMQTT();

  pinMode(hallSensorStationConnect, INPUT_PULLUP);
  pinMode(hallSensorBrakeConnect, INPUT_PULLUP);
  pinMode(hallSensorMiddle, INPUT_PULLUP);
  pinMode(hallSensorOnSwitchtrack, INPUT_PULLUP);

  rotatingStepper.setMaxSpeed(700);
  rotatingStepper.setAcceleration(400);

  tiltingStepper.setMaxSpeed(400);
  tiltingStepper.setAcceleration(200);

  releaseServo.attach(SERVO_PIN);
}

void loop()
{
  if (!client.connected())
    connectMQTT();
  client.loop();

  updateSensors();
  publishHeartbeat();
  publishStatusIfChanged();

  handleMovement();
  handleAutoControl();

  moveServoSmooth(releaseServo, currentPos, targetPos);
}
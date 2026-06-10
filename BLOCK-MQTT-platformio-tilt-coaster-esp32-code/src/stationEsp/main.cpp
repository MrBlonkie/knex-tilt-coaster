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
const char *deviceName = "station";
String clientId;
const char *connectivityTopic = "rollercoaster/station/status";

// === Heartbeat Config (NON-BLOCKING) ===
unsigned long lastHeartbeat = 0;
const long heartbeatInterval = 2500;

// === MQTT Retry Config ===
unsigned long lastMqttRetry = 0;
const unsigned long mqttRetryInterval = 2000;

// === Motor Config ===
// +++ Station +++
#define STATION_IN1 18
#define STATION_IN2 19
#define STATION_IN3 22
#define STATION_IN4 23

AccelStepper stationStepper(AccelStepper::FULL4WIRE, STATION_IN1, STATION_IN3, STATION_IN2, STATION_IN4);
bool stationStepperState = false;

#define SERVO_PIN 25
Servo enterServo;
int currentPos = 180;
int targetPos = 0; // Default Dicht
unsigned long lastStepEnter = 0;
const unsigned long stepInterval = 2;

unsigned long servoWaitTimer = 0;
bool isServoWaiting = false;

#define GATE_SERVO_PIN 15
Servo gatesServo;
int currentPos2 = 180;
int targetPos2 = 0;
bool gatesServoState = false;
unsigned long lastStepGates = 0;

// Non-blocking servo sweep
bool moveServoSmooth(Servo &servo, int &current, int target, unsigned long &lastStepVar)
{
  unsigned long now = millis();
  if (now - lastStepVar < stepInterval)
    return false;
  lastStepVar = now;

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

// +++ Lifthill +++
#define LIFT_DIR_PIN 26
#define LIFT_STEP_PIN 27
#define LIFT_ENABLE_PIN 14

AccelStepper liftStepper(AccelStepper::DRIVER, LIFT_STEP_PIN, LIFT_DIR_PIN);
bool liftStepperState = false;

// === Fan ===
#define RELAY_PIN 12
bool relayState = false;

// === Sensors ===
#define hallSensorExitStation 35
#define hallSensorBottomLifthill 33
#define hallSensorEnterStation 34
#define hallSensorStartPosition 32

bool hallSensorExitStationState = false;
bool hallSensorBottomLifthillState = false;
bool hallSensorEnterStationState = false;
bool hallSensorStartPositionState = false;

// === Helper Bools / State Control ===
bool coasterDispatched = false;
bool manualMode = false;

bool trainOnTiltdrop = false;

bool isStationOccupied = false;
bool isLifthillOccupied = false;
bool isNextBlockFree = false;

bool enterStationFlag = false;
bool startPositionFlag = false;
bool stationSafetyFlag = false;

unsigned long gatesOpenTimer = 0;
bool isGatesSequenceActive = false;
bool isGatesSequenceDone = false;
bool lifthillFlag = false;
bool lifthillSafetyFlag = false;
bool tiltdropFlag = false;

// === Helper motor stop ===
void StopStepperMotor(AccelStepper &motor)
{
  motor.stop();
  motor.setCurrentPosition(motor.currentPosition());
  motor.moveTo(motor.currentPosition());
}

// === MQTT callback ===
void callback(char *topic, byte *payload, unsigned int length)
{
  String message;
  for (int i = 0; i < length; i++)
    message += (char)payload[i];

  if (String(topic) == "station/manual" && message == "on")
  {
    manualMode = true;
  }
  else if (String(topic) == "station/manual" && message == "off")
  {
    manualMode = false;
    stationStepperState = false;
    liftStepperState = false;
    coasterDispatched = false; // nog niet op esp
  }
  else if (String(topic) == "rollercoaster/dispatch" && message == "go" && !manualMode)
  {
    coasterDispatched = true;
    Serial.println("recieved dispatch go command");
  }
  else if (String(topic) == "rollercoaster/dispatch" && message == "stop")
  {
    coasterDispatched = false;
    stationStepperState = false;
    liftStepperState = false;
  }
  else if (String(topic) == "station/stationmotor" && manualMode)
  {
    if (message == "on")
    {
      stationStepperState = true;
    }
    else
    {
      stationStepperState = false;
    }
  }
  else if (String(topic) == "station/lifthillmotor" && manualMode)
  {
    if (message == "on")
    {
      liftStepperState = true;
    }
    if (message == "off")
    {
      liftStepperState = false;
    }
  }
  else if (String(topic) == "station/gatesmotor" && manualMode)
  {
    if (message == "open")
    {
      // gates servo open
      targetPos2 = 3;
      gatesServoState = true;
    }
    else
    {
      // gates servo close
      targetPos2 = 108;
      gatesServoState = false;
    }
  }
  else if (String(topic) == "rollercoaster/control/auto")
  {
    if (message == "on")
      manualMode = false;
    else if (message == "off")
      manualMode = true;
  }
  if (String(topic) == "rollercoaster/event")
  {
    if (message == "train_on_tiltdrop")
    {
      trainOnTiltdrop = true; // <-- zet flag
    }
  }
  // Block System Events
  if (String(topic) == "rollercoaster/block/event")
  {
    if (message == "tiltdrop_free")
      isNextBlockFree = true;
    if (message == "tiltdrop_occupied")
      isNextBlockFree = false;
  }

  if (String(topic) == "rollercoaster/clear/station" && message == "clear")
  {
    stationSafetyFlag = false;
    client.publish("rollercoaster/event", "cleared_station_safety_flag");
  }
  if (String(topic) == "rollercoaster/clear/lifthill" && message == "clear")
  {
    lifthillSafetyFlag = false;
    client.publish("rollercoaster/event", "cleared_lifthill_safety_flag");
  }

  if (String(topic) == "rollercoaster/estop" && message == "stop")
  {
    stationStepperState = false;
    liftStepperState = false;
  }
}

// === MQTT connect zonder LWT, met Initiële Status op Heartbeat Topic ===
void connectMQTT()
{
  if (client.connected()) return;
  if (lastMqttRetry > 0 && millis() - lastMqttRetry < mqttRetryInterval) return;
  lastMqttRetry = millis();

  clientId = "roller-" + String(deviceName) + "-" + String((uint32_t)ESP.getEfuseMac());
  Serial.print("Connecting to MQTT...");
  if (client.connect(clientId.c_str()))
  {
    Serial.println("connected");

    client.publish(connectivityTopic, "online", true);

    client.subscribe("station/manual");
    client.subscribe("station/stationmotor");
    client.subscribe("station/lifthillmotor");
    client.subscribe("station/gatesmotor");

    client.subscribe("rollercoaster/control/auto");
    client.subscribe("rollercoaster/event");
    client.subscribe("rollercoaster/dispatch");

    client.subscribe("rollercoaster/block/event");
    client.subscribe("rollercoaster/estop");
    client.subscribe("rollercoaster/reset");
    client.subscribe("rollercoaster/clear/station");
    client.subscribe("rollercoaster/clear/lifthill");
  }
  else
  {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(", retrying in 2s...");
  }
}

// === Sensors update ===
void updateSensors()
{
  hallSensorExitStationState = digitalRead(hallSensorExitStation) == LOW;
  hallSensorBottomLifthillState = digitalRead(hallSensorBottomLifthill) == LOW;
  hallSensorEnterStationState = digitalRead(hallSensorEnterStation) == LOW;
  hallSensorStartPositionState = digitalRead(hallSensorStartPosition) == LOW;
}

void handleStationBlockV2()
{

  bool stationMovementAllowed = coasterDispatched && !stationSafetyFlag;

  // Start after stop Logic
  if (isStationOccupied && stationMovementAllowed)
  {
    if (!(hallSensorStartPositionState && isLifthillOccupied) && !isGatesSequenceActive && !isGatesSequenceDone)
    {
      stationStepperState = true;
    }
  }

  // Enter Station Logic
  if (hallSensorEnterStationState && !enterStationFlag && stationMovementAllowed)
  {
    stationStepperState = true;
    enterStationFlag = true;
    isStationOccupied = true;
    servoWaitTimer = millis();
    isServoWaiting = true;
    client.publish("rollercoaster/event", "train_enters_station");
    client.publish("rollercoaster/block/event", "station_occupied");
  }

  // timing enterServo Logic
  if (isServoWaiting && (millis() - servoWaitTimer >= 200))
  {
    targetPos = 180;        // Beweeg naar 90 graden na 0.2s
    isServoWaiting = false; // Stop met wachten
  }

  // Start Position Logic - trigger once on arrival
  if (hallSensorStartPositionState && !startPositionFlag)
  {
    stationStepperState = false;
    startPositionFlag = true;
    targetPos2 = 3;
    gatesServoState = true;
    gatesOpenTimer = millis();
    isGatesSequenceActive = true;
    isGatesSequenceDone = false;

    if (targetPos != 0)
      targetPos = 0;
  }

  // Close gates after delay
  if (isGatesSequenceActive && (millis() - gatesOpenTimer >= 700))
  {
    targetPos2 = 108;
    gatesServoState = false;
    isGatesSequenceActive = false;
    isGatesSequenceDone = true;
  }

  // Restart motor after gates closed
  if (isGatesSequenceDone && hallSensorStartPositionState)
  {
    if (!isLifthillOccupied && stationMovementAllowed)
    {
      stationStepperState = true;
      isGatesSequenceDone = false;
      client.publish("rollercoaster/event", "train_moves_to_lifthill");
    }
    else if (isLifthillOccupied)
    {
      stationStepperState = false;
      client.publish("rollercoaster/event", "train_waits_in_start_position");
    }
  }

  // exit logic
  if (hallSensorExitStationState)
  {
    stationStepperState = false;
    enterStationFlag = false;
    startPositionFlag = false;
    isGatesSequenceActive = false;
    isGatesSequenceDone = false;
    isStationOccupied = false;
    client.publish("rollercoaster/event", "train_left_station");
    client.publish("rollercoaster/block/event", "station_free");
  }
}

void handleLifthillBlockV2()
{

  bool lifthillMovementAllowed = coasterDispatched && isNextBlockFree && !lifthillSafetyFlag;

  // enter lifthill
  if (hallSensorBottomLifthillState && !lifthillFlag)
  {
    lifthillFlag = true;
    isLifthillOccupied = true;
    client.publish("rollercoaster/event", "train_on_lifthill");
    client.publish("rollercoaster/block/event", "lifthill_occupied");
  }

  // lifthill on
  if (isLifthillOccupied)
  {
    if (lifthillMovementAllowed)
    {
      liftStepperState = true;
      tiltdropFlag = false;
    }
    else
    {
      liftStepperState = false;
    }
  }

  // turn off lifthill
  if (trainOnTiltdrop && !tiltdropFlag)
  {
    liftStepperState = false;
    tiltdropFlag = true;
    isLifthillOccupied = false;
    lifthillFlag = false;
    trainOnTiltdrop = false;
    client.publish("rollercoaster/event", "train_exits_lifthill");
    client.publish("rollercoaster/block/event", "lifthill_free");
  }

  // safety
  if (liftStepperState && !isNextBlockFree)
  {
    liftStepperState = false;
    lifthillSafetyFlag = true;
    client.publish("rollercoaster/event", "lifthill_stopped_and_train_on_lifthill_____clear_lifthill_to_continue");
  }
}

void handleMotorControl()
{
  if (stationStepperState)
  {
    stationStepper.setSpeed(600);
    stationStepper.runSpeed();
  }
  else
  {
    StopStepperMotor(stationStepper);
  }

  if (liftStepperState)
  {
    digitalWrite(LIFT_ENABLE_PIN, LOW);

    // Fan Logic
    if (!relayState)
    {
      digitalWrite(RELAY_PIN, HIGH);
      relayState = true;
    }

    liftStepper.setSpeed(600);
    liftStepper.runSpeed();
  }
  else
  {
    StopStepperMotor(liftStepper);
    digitalWrite(LIFT_ENABLE_PIN, HIGH);

    // Fan Logic
    if (relayState)
    {
      digitalWrite(RELAY_PIN, LOW);
      relayState = false;
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
    Serial.println("[HB] Heartbeat sent: online");
  }
}

// === Publish Status via MQTT (Gedetailleerde JSON) ===
String lastStatus = "";
void publishStatusIfChanged()
{
  String status = "{";
  status += "\"sensors\":{";
  status += "\"hallSensorExitStationState\":" + String(hallSensorExitStationState ? "true" : "false");
  status += ",\"hallSensorBottomLifthillState\":" + String(hallSensorBottomLifthillState ? "true" : "false");
  status += ",\"hallSensorEnterStationState\":" + String(hallSensorEnterStationState ? "true" : "false");
  status += ",\"hallSensorStartPositionState\":" + String(hallSensorStartPositionState ? "true" : "false");
  status += "},";

  status += "\"coaster\":{";
  status += "\"coasterDispatched\":" + String(coasterDispatched ? "true" : "false");
  status += "},";

  status += "\"mode\":{";
  status += "\"manualMode\":" + String(manualMode ? "true" : "false");
  status += "},";

  status += "\"motors\":{";
  status += "\"station\":{";
  status += "\"stationStepperState\":" + String(stationStepperState ? "true" : "false");
  status += ",\"gatesServoState\":" + String(gatesServoState ? "true" : "false");
  status += "},";
  status += "\"lift\":{";
  status += "\"liftStepperState\":" + String(liftStepperState ? "true" : "false");
  status += "}";
  status += "},";

  status += "\"blocks\":{";
  status += "\"isStationOccupied\":" + String(isStationOccupied ? "true" : "false");
  status += ",\"isLifthillOccupied\":" + String(isLifthillOccupied ? "true" : "false");
  status += ",\"isNextBlockFree\":" + String(isNextBlockFree ? "true" : "false");
  status += "},";

  status += "\"flags\":{";
  status += "\"enterStationFlag\":" + String(enterStationFlag ? "true" : "false");
  status += ",\"startPositionFlag\":" + String(startPositionFlag ? "true" : "false");
  status += ",\"stationSafetyFlag\":" + String(stationSafetyFlag ? "true" : "false");
  status += ",\"lifthillFlag\":" + String(lifthillFlag ? "true" : "false");
  status += ",\"lifthillSafetyFlag\":" + String(lifthillSafetyFlag ? "true" : "false");
  status += ",\"tiltdropFlag\":" + String(tiltdropFlag ? "true" : "false");
  status += "}";
  status += "}";

  if (status != lastStatus)
  {
    client.publish("station/status", status.c_str(), true);
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
  {
    unsigned long t = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
      if (millis() - t >= 300) { t = millis(); Serial.print("."); }
    }
  }
  Serial.println("Connected! IP: " + WiFi.localIP().toString());

  // MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  connectMQTT();

  // LED
  FastLED.addLeds<NEOPIXEL, LEDS_EXIT_PIN>(exitLeds, NUM_LEDS_EXIT);

  // Sensors
  pinMode(hallSensorExitStation, INPUT_PULLUP);
  pinMode(hallSensorBottomLifthill, INPUT_PULLUP);
  pinMode(hallSensorEnterStation, INPUT_PULLUP);
  pinMode(hallSensorStartPosition, INPUT_PULLUP);

  // Motors
  stationStepper.setMaxSpeed(800.0);
  stationStepper.setAcceleration(0);

  enterServo.attach(SERVO_PIN);
  gatesServo.attach(GATE_SERVO_PIN);

  liftStepper.setMaxSpeed(800.0);
  liftStepper.setAcceleration(400.0);

  pinMode(LIFT_ENABLE_PIN, OUTPUT);
  digitalWrite(LIFT_ENABLE_PIN, HIGH);

  pinMode(RELAY_PIN, OUTPUT);
}

// === Loop ===
void loop()
{
  if (!client.connected())
    connectMQTT();
  client.loop();

  updateSensors();
  publishHeartbeat();
  handleMotorControl();
  moveServoSmooth(enterServo, currentPos, targetPos, lastStepEnter);
  moveServoSmooth(gatesServo, currentPos2, targetPos2, lastStepGates);

  if (coasterDispatched)
  {
    // Automatische bediening
    handleStationBlockV2();
    handleLifthillBlockV2();
  }

  // LED updates
  if (stationStepperState)
  {
    UpdateLedFadeMotor();
  }
  else
  {
    UpdateLedFadeIdle();
  }

  publishStatusIfChanged();
}
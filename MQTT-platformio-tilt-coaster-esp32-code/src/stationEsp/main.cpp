#include <WiFi.h>
#include <PubSubClient.h>
#include <AccelStepper.h>
#include <FastLED.h>
#include "LedSetup.h"
#include "../shared/env.h" // ssid & password, mqtt_server, mqtt_port

// === WiFi & MQTT ===
WiFiClient espClient;
PubSubClient client(espClient);
const char *deviceName = "station";
String clientId;
// De topic voor de connectiviteitsstatus (Heartbeat)
const char *connectivityTopic = "rollercoaster/station/status";

// === Heartbeat Config (NON-BLOCKING) ===
unsigned long lastHeartbeat = 0;
const long heartbeatInterval = 2500; // 10 seconden

// === Motor Config ===
// station
#define STATION_IN1 18
#define STATION_IN2 19
#define STATION_IN3 22
#define STATION_IN4 23

AccelStepper stationStepper(AccelStepper::FULL4WIRE, STATION_IN1, STATION_IN3, STATION_IN2, STATION_IN4);
bool stationStepperState = false;

// lifthill
#define LIFT_DIR_PIN 26
#define LIFT_STEP_PIN 27
#define LIFT_ENABLE_PIN 14

AccelStepper liftStepper(AccelStepper::DRIVER, LIFT_STEP_PIN, LIFT_DIR_PIN);
bool liftStepperState = false;

// === Sensors ===
#define hallSensorExitStation 35
#define hallSensorBottomLifthill 33
#define hallSensorEnterStation 34
#define hallSensorStartPosition 32

bool hallSensorExitStationState = false;
bool hallSensorBottomLifthillState = false;
bool hallSensorEnterStationState = false;
bool hallSensorStartPositionState = false;

bool trainOnTiltdrop = false;
bool autoMode = false;

// === States ===
enum CoasterState
{
  STATE_IDLE,
  STATE_DISPATCHING,
  STATE_TO_LIFTHILL,
  STATE_CLIMBING,
  STATE_RIDING,
  STATE_ENTER_STATION,
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
  case STATE_DISPATCHING:
    currentStateName = "DISPATCHING";
    break;
  case STATE_TO_LIFTHILL:
    currentStateName = "TO_LIFTHILL";
    break;
  case STATE_CLIMBING:
    currentStateName = "CLIMBING";
    break;
  case STATE_RIDING:
    currentStateName = "RIDING";
    break;
  case STATE_ENTER_STATION:
    currentStateName = "ENTER_STATION";
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
bool stationMotorManual = false;
bool liftMotorManual = false;

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
    stationStepper.stop();
    liftStepper.stop();
    stationStepperState = false;
    liftStepperState = false;
  }
  else if (String(topic) == "rollercoaster/dispatch" && message == "go")
  {
    coasterDispatched = true;
    Serial.println("recieved dispatch go command");
  }
  else if (String(topic) == "station/stationmotor" && manualMode)
  {
    if (message == "on")
    {
      stationStepper.setSpeed(600);
      stationStepperState = true;
    }
    else
    {
      stationStepper.stop();
      stationStepperState = false;
    }
  }
  else if (String(topic) == "station/lifthillmotor" && manualMode)
  {
    if (message == "on")
    {
      digitalWrite(LIFT_ENABLE_PIN, LOW);
      liftStepper.setSpeed(600);
      liftStepperState = true;
    }
    else
    {
      liftStepper.stop();
      digitalWrite(LIFT_ENABLE_PIN, HIGH);
      liftStepperState = false;
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
}

// === MQTT connect zonder LWT, met Initiële Status op Heartbeat Topic ===
void connectMQTT()
{
  clientId = "roller-" + String(deviceName) + "-" + String((uint32_t)ESP.getEfuseMac());

  while (!client.connected())
  {
    Serial.print("Connecting to MQTT...");
    // Geen LWT meer, dus client.connect() zonder extra parameters
    if (client.connect(clientId.c_str()))
    {
      Serial.println("connected");

      // Publiceer online status op het specifieke connectiviteit-topic (retained)
      client.publish(connectivityTopic, "online", true);

      // Subscribe naar control topics
      client.subscribe("station/manual");
      client.subscribe("station/dispatch");
      client.subscribe("station/stationmotor");
      client.subscribe("station/lifthillmotor");
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

// === Helper motor stop ===
void StopStepperMotor(AccelStepper &motor)
{
  motor.stop();
  motor.setCurrentPosition(motor.currentPosition());
  motor.moveTo(motor.currentPosition());
}

// === Sensors update ===
void updateSensors()
{
  hallSensorExitStationState = digitalRead(hallSensorExitStation) == LOW;
  hallSensorBottomLifthillState = digitalRead(hallSensorBottomLifthill) == LOW;
  hallSensorEnterStationState = digitalRead(hallSensorEnterStation) == LOW;
  hallSensorStartPositionState = digitalRead(hallSensorStartPosition) == LOW;
}

// AUTO LOGIC
void handleAutoControl()
{
  switch (currentState)
  {
  case STATE_IDLE:
    // Simuleer dispatch start
    if (coasterDispatched)
    {
      coasterDispatched = false;
      stationStepper.move(10000);
      setState(STATE_DISPATCHING);
      client.publish("rollercoaster/event", "train_dispatched");
    }
    break;

  case STATE_DISPATCHING:
    stationStepper.run();
    if (hallSensorExitStationState)
    {
      StopStepperMotor(stationStepper);
      setState(STATE_TO_LIFTHILL);
      client.publish("rollercoaster/event", "train_left_station");
    }
    break;

  case STATE_TO_LIFTHILL:
    if (hallSensorBottomLifthillState)
    {
      client.publish("rollercoaster/event", "train_on_lifthill");
      // lifthill stepper logic
      setState(STATE_CLIMBING);
    }
    break;

  case STATE_CLIMBING:
    if (trainOnTiltdrop)
    {
      trainOnTiltdrop = false;
      //StopStepperMotor(liftStepper);
      client.publish("rollercoaster/event", "train_on_tiltdrop");
      setState(STATE_RIDING);
    }
    break;

  case STATE_RIDING:
    if (hallSensorEnterStationState)
    {
      client.publish("rollercoaster/event", "train_enters_station");
      setState(STATE_ENTER_STATION);
    }
    break;

  case STATE_ENTER_STATION:
    stationStepper.move(10000);
    if (hallSensorStartPositionState)
    {
      StopStepperMotor(stationStepper);
      client.publish("rollercoaster/event", "train_in_start_position");
      setState(STATE_IDLE);
    }
    break;
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
  status += "\"hallSensorExitStation\":" + String(hallSensorExitStationState ? "true" : "false");
  status += ",\"hallSensorBottomLifthill\":" + String(hallSensorBottomLifthillState ? "true" : "false");
  status += ",\"hallSensorEnterStation\":" + String(hallSensorEnterStationState ? "true" : "false");
  status += ",\"hallSensorStartPosition\":" + String(hallSensorStartPositionState ? "true" : "false");
  status += ",\"coasterDispatched\":" + String(coasterDispatched ? "true" : "false");
  status += ",\"manualMode\":" + String(manualMode ? "true" : "false");
  status += ",\"stationMotorManual\":" + String(stationMotorManual ? "true" : "false");
  status += ",\"stationMotorState\":" + String(stationStepperState ? "true" : "false");
  status += ",\"liftMotorManual\":" + String(liftMotorManual ? "true" : "false");
  status += ",\"liftMotorState\":" + String(liftStepperState ? "true" : "false");
  status += ",\"currentState\":\"" + currentStateName + "\"";
  status += "}";

  // Dit is de gedetailleerde status op de oorspronkelijke topic
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

  // LED
  FastLED.addLeds<NEOPIXEL, LEDS_EXIT_PIN>(exitLeds, NUM_LEDS_EXIT);

  // Sensors
  pinMode(hallSensorExitStation, INPUT_PULLUP);
  pinMode(hallSensorBottomLifthill, INPUT_PULLUP);
  pinMode(hallSensorEnterStation, INPUT_PULLUP);
  pinMode(hallSensorStartPosition, INPUT_PULLUP);

  // Motors
  stationStepper.setMaxSpeed(800.0);
  stationStepper.setAcceleration(300.0);

  liftStepper.setMaxSpeed(800.0);
  liftStepper.setAcceleration(400.0);

  pinMode(LIFT_ENABLE_PIN, OUTPUT);
  digitalWrite(LIFT_ENABLE_PIN, HIGH);

  // States
  setState(STATE_IDLE);
}

// === Loop ===
void loop()
{
  // Dit moet altijd draaien
  if (!client.connected())
    connectMQTT();
  client.loop();

  updateSensors();

  // NON-BLOCKING Heartbeat
  publishHeartbeat();

  if (manualMode)
  {
    if (stationStepperState)
      stationStepper.runSpeed();
    if (liftStepperState)
      liftStepper.runSpeed();
  }

  if (!manualMode)
  {
    handleAutoControl();
  }
  stationStepper.run();
  liftStepper.run();

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
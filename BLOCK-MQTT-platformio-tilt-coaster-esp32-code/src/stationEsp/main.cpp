#include <WiFi.h>
#include <PubSubClient.h>
#include <AccelStepper.h>
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

// === Motor Config ===
// +++ Station +++
#define STATION_IN1 18
#define STATION_IN2 19
#define STATION_IN3 22
#define STATION_IN4 23

AccelStepper stationStepper(AccelStepper::FULL4WIRE, STATION_IN1, STATION_IN3, STATION_IN2, STATION_IN4);
bool stationStepperState = false;

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
bool stationMotorManual = false;
bool liftMotorManual = false;

bool trainOnTiltdrop = false;

bool isStationOccupied = false;
bool isLifthillOccupied = false;
bool isNextBlockFree = false;


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
  }
  else if (String(topic) == "rollercoaster/dispatch" && message == "go" && !manualMode)
  {
    coasterDispatched = true;
    Serial.println("recieved dispatch go command");
  }
  else if (String(topic) == "rollercoaster/dispatch" && message == "stop")
  {
    coasterDispatched = false;
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
    else
    {
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
    if (message == "tiltdrop_closed")
    {
      trainOnTiltdrop = false;
      
      isLifthillOccupied = false; 
      client.publish("rollercoaster/block/event", "lifthill_free");
    }
  }
  // Block System Events
    if (String(topic) == "rollercoaster/block/event") {
        if (message == "tiltdrop_free") isNextBlockFree = true;
        if (message == "tiltdrop_occupied") isNextBlockFree = false;
    }

  if(String(topic) == "rollercoaster/reset/station" && message == "reset")
  {
      isStationOccupied = false;
      isLifthillOccupied = false;
      isNextBlockFree = true;
  }

  if(String(topic) == "rollercoaster/estop" && message == "stop")
  {
    stationStepperState = false;
    liftStepperState = false;
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
      client.subscribe("rollercoaster/sensor");

      client.subscribe("rollercoaster/block/event");
      client.subscribe("rollercoaster/estop");
      client.subscribe("rollercoaster/reset");
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

// === Sensors update ===
void updateSensors()
{
  hallSensorExitStationState = digitalRead(hallSensorExitStation) == LOW;
  hallSensorBottomLifthillState = digitalRead(hallSensorBottomLifthill) == LOW;
  hallSensorEnterStationState = digitalRead(hallSensorEnterStation) == LOW;
  hallSensorStartPositionState = digitalRead(hallSensorStartPosition) == LOW;
}

void handleStationBlock() {
  if (hallSensorEnterStationState && !hallSensorStartPositionState) {
     stationStepperState = true; 
  }

  if (hallSensorStartPositionState && !isStationOccupied) {
     stationStepperState = false; 
 
     client.publish("rollercoaster/block/event", "station_occupied");
  }

  if (isStationOccupied && !isLifthillOccupied) {
     stationStepperState = true;
  }

  if (hallSensorExitStationState && isStationOccupied) {
     isStationOccupied = false; 
     stationStepperState = false; 
     coasterDispatched = false;   
     
     client.publish("rollercoaster/block/event", "station_free");
     Serial.println("BLOCK: Station Free");
  }
}

void handleLifthillBlock() {
  if (hallSensorBottomLifthillState && !isLifthillOccupied) {
     isLifthillOccupied = true;
     client.publish("rollercoaster/block/event", "lifthill_occupied");
     Serial.println("BLOCK: Lifthill Occupied");
  }

  if (isLifthillOccupied) {
    if (trainOnTiltdrop && !isNextBlockFree) {
       liftStepperState = false; 
    } else {
       liftStepperState = true; 
    }
  } else {
    liftStepperState = false;
  }
  if (!trainOnTiltdrop && !isLifthillOccupied) {
     liftStepperState = false;
  }
}

void handleMotorControl(){
  if(stationStepperState)
  {
    stationStepper.setSpeed(600);
    stationStepper.runSpeed();
  }
  else
  {
    StopStepperMotor(stationStepper);
  }

  if(liftStepperState)
  {
    digitalWrite(LIFT_ENABLE_PIN, LOW);
    
    // Fan Logic
    if (!relayState) {
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
    if (relayState) {
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
  status += ",\"relayState\":" + String(relayState ? "true" : "false");
  status += ",\"isStationOccupied\":" + String(isStationOccupied ? "true" : "false");
  status += ",\"isLifthillOccupied\":" + String(isLifthillOccupied ? "true" : "false");
  status += ",\"isNextBlockFree\":" + String(isNextBlockFree ? "true" : "false");
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
  stationStepper.setAcceleration(0);

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

  if(coasterDispatched)
  {
    // Automatische bediening
    handleStationBlock();
    handleLifthillBlock();
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
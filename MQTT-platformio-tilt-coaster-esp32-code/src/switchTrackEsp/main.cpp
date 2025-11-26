#include <AccelStepper.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "../shared/env.h" // ssid, password, mqtt_server, mqtt_portt

// === WiFi & MQTT ===
WiFiClient espClient;
PubSubClient client(espClient);
const char *deviceName = "switchtrack";
String clientId;
// De topic voor de connectiviteitsstatus (Heartbeat)
const char *connectivityTopic = "rollercoaster/switchtrack/status";

// === Heartbeat Config (NON-BLOCKING) ===
unsigned long lastHeartbeat = 0;
// Heartbeat elke 2,5 seconden (voor 4,5s detectie)
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

#define hallSensorStationConnect 27
#define hallSensorBrakeConnect 26
#define hallSensorMiddle 25

#define controlServoPin 33

AccelStepper rotatingStepper(AccelStepper::FULL4WIRE, ROTATING_IN1, ROTATING_IN3, ROTATING_IN2, ROTATING_IN4);
AccelStepper tiltingStepper(AccelStepper::FULL4WIRE, TILTING_IN1, TILTING_IN3, TILTING_IN2, TILTING_IN4);

bool hallSensorStationConnectState = false;
bool hallSensorMiddleState = false;
bool hallSensorBrakeConnectState = false;

// === MQTT callback ===
void callback(char *topic, byte *payload, unsigned int length)
{
  String message;
  for (int i = 0; i < length; i++)
    message += (char)payload[i];
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

// === Sensors update ===
void updateSensors()
{
  hallSensorBrakeConnectState = digitalRead(hallSensorBrakeConnect) == LOW;
  hallSensorMiddleState = digitalRead(hallSensorMiddle) == LOW;
  hallSensorStationConnectState = digitalRead(hallSensorStationConnect) == LOW;
}

// === Publish Status via MQTT (Gedetailleerde JSON) ===
String lastStatus = "";
void publishStatusIfChanged()
{
  String status = "{";
  status += "\"hallSensorBrakeConnect\":" + String(hallSensorBrakeConnectState ? "true" : "false");
  status += ",\"hallSensorMiddle\":" + String(hallSensorMiddleState ? "true" : "false");
  status += ",\"hallSensorStationConnect\":" + String(hallSensorStationConnectState ? "true" : "false");
  status += "}";

  // Dit is de gedetailleerde status op de oorspronkelijke topic
  if (status != lastStatus)
  {
    client.publish("switchtrack/status", status.c_str(), true);
    lastStatus = status;
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
      client.subscribe("switchtrack/manual");
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

  pinMode(hallSensorStationConnect, INPUT_PULLUP);
  pinMode(hallSensorBrakeConnect, INPUT_PULLUP);
  pinMode(hallSensorMiddle, INPUT_PULLUP);

  rotatingStepper.setMaxSpeed(700);
  rotatingStepper.setAcceleration(400);

  tiltingStepper.setMaxSpeed(700);
  tiltingStepper.setAcceleration(400);
}

void loop()
{
  if (!client.connected())
    connectMQTT();
  client.loop();

  updateSensors();

  publishHeartbeat();

  publishStatusIfChanged();
}

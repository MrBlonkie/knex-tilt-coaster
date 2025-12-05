#include <AccelStepper.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "../shared/env.h" // Zorg dat ssid, password, mqtt_server, etc hierin staan

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

#define hallSensorStationConnect 27
#define hallSensorBrakeConnect 26
#define hallSensorMiddle 25
#define hallSensorOnSwitchtrack 32

AccelStepper rotatingStepper(AccelStepper::FULL4WIRE, ROTATING_IN1, ROTATING_IN3, ROTATING_IN2, ROTATING_IN4);
// Tilting stepper wordt hier geïnitialiseerd maar niet gebruikt in de callback logica
AccelStepper tiltingStepper(AccelStepper::FULL4WIRE, TILTING_IN1, TILTING_IN3, TILTING_IN2, TILTING_IN4);

bool hallSensorStationConnectState = false;
bool hallSensorMiddleState = false;
bool hallSensorBrakeConnectState = false;
bool hallSensorOnSwitchtrackState = false;

bool manualMode = false;
bool coasterDispatched = false;

// === MQTT callback ===
void callback(char *topic, byte *payload, unsigned int length)
{
  String message;
  for (int i = 0; i < length; i++)
    message += (char)payload[i];

  // --- DEBUG: Print ALLES wat binnenkomt direct ---
  Serial.print("DEBUG [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);
  // ------------------------------------------------

  // Logica
  if (String(topic) == "switchtrack/manual")
  {
    if (message == "on") {
      manualMode = true;
      Serial.println("Manual mode ENABLED");
    } else if (message == "off") {
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
      if (message == "on") Serial.println("COMMAND: Rotate to Brakes");
      if (message == "off") Serial.println("COMMAND: Rotate to Station");
    }
    else if (String(topic) == "switchtrack/releaseswitchtrackmotor")
    {
      if (message == "open") Serial.println("COMMAND: Servo Open");
      if (message == "close") Serial.println("COMMAND: Servo Close");
    }
  }
  else 
  {
    // Als we hier komen, is er wel een commando, maar staat manual mode uit
    if (String(topic).startsWith("switchtrack/")) {
       Serial.println("IGNORED: Manual mode is OFF");
    }
  }
}

// === Publish Heartbeat ===
void publishHeartbeat()
{
  if (millis() - lastHeartbeat >= heartbeatInterval)
  {
    lastHeartbeat = millis();
    client.publish(connectivityTopic, "online", true);
    // Serial.println("[HB] Heartbeat sent"); // Commentaar weggehaald om serial schoon te houden voor testen
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
  status += ",\"manualMode\":" + String(manualMode ? "true" : "false");
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

      // Subscribe topics
      client.subscribe("switchtrack/manual");
      client.subscribe("switchtrack/rotatemotor");
      client.subscribe("switchtrack/releaseswitchtrackmotor"); // Let op de spelling!
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
  connectMQTT();

  pinMode(hallSensorStationConnect, INPUT_PULLUP);
  pinMode(hallSensorBrakeConnect, INPUT_PULLUP);
  pinMode(hallSensorMiddle, INPUT_PULLUP);
  pinMode(hallSensorOnSwitchtrack, INPUT_PULLUP);

  rotatingStepper.setMaxSpeed(700);
  rotatingStepper.setAcceleration(400);

  tiltingStepper.setMaxSpeed(700);
  tiltingStepper.setAcceleration(400);
}

void loop()
{
  if (!client.connected()) connectMQTT();
  client.loop();

  updateSensors();
  publishHeartbeat();
  publishStatusIfChanged();
  
  // Voeg later hier rotatingStepper.run() toe als je wilt bewegen
}
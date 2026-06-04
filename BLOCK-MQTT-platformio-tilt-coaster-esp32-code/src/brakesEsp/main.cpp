#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include "../shared/env.h"

// === WiFi & MQTT ===
WiFiClient espClient;
PubSubClient client(espClient);
const char *deviceName = "brakes";
String clientId;
const char *connectivityTopic = "rollercoaster/brakes/status";

// === Heartbeat Config ===
unsigned long lastHeartbeat = 0;
const long heartbeatInterval = 2500;

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

// === Sensor config ===
#define hallSensorExitBrakes 27
#define hallSensorEnterBrakes 26

bool hallSensorExitBrakesState = false;
bool hallSensorEnterBrakesState = false;

void updateSensors()
{
    hallSensorEnterBrakesState = digitalRead(hallSensorEnterBrakes) == LOW;
    hallSensorExitBrakesState = digitalRead(hallSensorExitBrakes) == LOW;
}

// === Servo config ===
#define SERVO_PIN 25
Servo releaseServo;
bool releaseBrakesMotorState = false;

int currentPos = 0;
int targetPos = 90;
unsigned long lastServoStep = 0;
const unsigned long stepInterval = 2;

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

// === Flags ===
bool manualMode = false;
bool coasterDispatched = false;
bool isNextBlockFree = false;
bool isBrakesOccupied = false;
bool brakesFlag = false;
bool isLayoutFree = false;

// === MQTT callback ===
void callback(char *topic, byte *payload, unsigned int length)
{
    String message;
    for (int i = 0; i < length; i++)
        message += (char)payload[i];

    if (String(topic) == "brakes/manual")
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
    else if (manualMode)
    {
        if (String(topic) == "brakes/releasebrakesmotor")
        {
            if (message == "open")
            {
                Serial.println("COMMAND: Servo Open");
                targetPos = 0;
                releaseBrakesMotorState = true;
            }
            if (message == "close")
            {
                Serial.println("COMMAND: Servo Close");
                targetPos = 90;
                releaseBrakesMotorState = false;
            }
        }
    }

    // Block System Events
  if (String(topic) == "rollercoaster/block/event")
  {
    if (message == "switchtrack_free")
      isNextBlockFree = true;
    if (message == "switchtrack_occupied")
      isNextBlockFree = false;
    if (message == "layout_free")
      isLayoutFree = true;
    if (message == "layout_occupied")
      isLayoutFree = false;
  }

  if (String(topic) == "rollercoaster/clear/brakes" && message == "clear")
    {
        isBrakesOccupied = false;
        client.publish("rollercoaster/block/event", "brakes_free");
        client.publish("rollercoaster/block/event", "layout_free");
    }

  if (String(topic) == "rollercoaster/estop" && message == "stop")
  {
    targetPos = 90;
    releaseBrakesMotorState = false;
    coasterDispatched = false;
    client.publish("rollercoaster/event", "estop_brakes");
  }
}

// === Publish Status ===
String lastStatus = "";
void publishStatusIfChanged()
{
    String status = "{";

    status += "\"sensors\":{";
    status += "\"hallSensorEnterBrakes\":" + String(hallSensorEnterBrakesState ? "true" : "false");
    status += ",\"hallSensorExitBrakes\":" + String(hallSensorExitBrakesState ? "true" : "false");
    status += "},";

    status += "\"mode\":{";
    status += "\"manualMode\":" + String(manualMode ? "true" : "false");
    status += "},";

    status += "\"motors\":{";
    status += "\"releaseBrakesMotorState\":" + String(releaseBrakesMotorState ? "true" : "false");
    status += "},";

    status += "\"blocks\":{";
    status += "\"isBrakesOccupied\":" + String(isBrakesOccupied ? "true" : "false");
    status += ",\"isNextBlockFree\":" + String(isNextBlockFree ? "true" : "false");
    status += ",\"isLayoutFree\":" + String(isLayoutFree ? "true" : "false");
    status += "}";

    status += "}";

    if (status != lastStatus)
    {
        client.publish("brakes/status", status.c_str(), true);
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

            client.subscribe("brakes/manual");
            client.subscribe("rollercoaster/control/auto");
            client.subscribe("rollercoaster/event");
            client.subscribe("rollercoaster/dispatch");

            client.subscribe("brakes/releasebrakesmotor");

            client.subscribe("rollercoaster/block/event");
            client.subscribe("rollercoaster/clear/brakes");
            client.subscribe("rollercoaster/estop");
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

void handleBrakesBlock() {
    
    if(hallSensorEnterBrakesState && !brakesFlag){
        isBrakesOccupied = true;
        client.publish("rollercoaster/event", "train_on_brakes");
        client.publish("rollercoaster/block/event", "brakes_occupied");
    }

    if(isNextBlockFree && !brakesFlag && isBrakesOccupied){
        targetPos = 0;
        releaseBrakesMotorState = true;
        brakesFlag = true;
        client.publish("rollercoaster/event", "releasing_brakes");
    }

    if(hallSensorExitBrakesState && brakesFlag && isBrakesOccupied){
        targetPos = 90;
        releaseBrakesMotorState = false;
        brakesFlag = false;
        isBrakesOccupied = false;
        client.publish("rollercoaster/event", "train_off_brakes");
        client.publish("rollercoaster/block/event", "brakes_free");
        client.publish("rollercoaster/block/event", "layout_free");
    }
}

void setup()
{
    Serial.begin(9600);

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

    pinMode(hallSensorExitBrakes, INPUT_PULLUP);
    pinMode(hallSensorEnterBrakes, INPUT_PULLUP);

    releaseServo.attach(SERVO_PIN);
}

void loop()
{
    if (!client.connected())
        connectMQTT();
    client.loop();

    publishHeartbeat();
    publishStatusIfChanged();
    updateSensors();

    moveServoSmooth(releaseServo, currentPos, targetPos);

    if(!manualMode){
        handleBrakesBlock();
    }
}
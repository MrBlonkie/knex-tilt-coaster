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

bool manualMode = false;

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

            client.subscribe("rollercoaster/block/event");
            client.subscribe("rollercoaster/clear/switchtrack");
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
    client.setKeepAlive(30);
    connectMQTT();

    pinMode(hallSensorExitBrakes, INPUT_PULLUP);
    pinMode(hallSensorEnterBrakes, INPUT_PULLUP);
}

void loop()
{
    if (!client.connected())
        connectMQTT();
    client.loop();

    publishHeartbeat();
    publishStatusIfChanged();
    updateSensors();
}
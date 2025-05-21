#include <WiFi.h>
#include <esp_now.h>
#include <Stepper.h>
#include <ESP32Servo.h>
#include "Message.h"

#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23
#define SERVO_PIN 25

#define hallSensorOnTiltdrop     32
#define hallSensorTiltdropClosed 27
#define hallSensorTiltdropOpen   34
#define hallSensorOffTiltdrop    26

const int stepsTiltTrack = 550;
Stepper tiltTrackStepper(stepsTiltTrack, IN1, IN3, IN2, IN4);
Servo releaseServo;

uint8_t masterAddress[] = {0x14, 0x2B, 0x2F, 0xC9, 0x24, 0xCC}; // Vervang door *jouw* master-MAC

volatile bool newCommandAvailable = false;
struct_message incomingCommand;

// Callback
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  Serial.print("ESP-NOW callback ontvangen. Lengte = ");
  Serial.println(len);

  Serial.print("Van MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(info->src_addr[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  if (len == sizeof(incomingCommand)) {
    memcpy(&incomingCommand, data, sizeof(incomingCommand));
    Serial.print("Inhoud commandostruct: ");
    Serial.println(incomingCommand.text);
    newCommandAvailable = true;
  } else {
    Serial.println("⚠️ Ongeldige lengte ontvangen voor commandostruct.");
  }
}

bool receiveCommand(char* buffer, size_t bufferSize) {
  if (!newCommandAvailable) return false;

  Serial.println("Nieuw commando beschikbaar! Kopieer naar buffer...");
  strncpy(buffer, incomingCommand.text, bufferSize - 1);
  buffer[bufferSize - 1] = '\0';
  newCommandAvailable = false;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== TILTDROP SLAVE SETUP ===");

  WiFi.mode(WIFI_STA);
  Serial.print("Mijn MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init mislukt!");
    return;
  } else {
    Serial.println("✅ ESP-NOW init geslaagd.");
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t masterInfo = {};
  memcpy(masterInfo.peer_addr, masterAddress, 6);
  masterInfo.channel = 0;
  masterInfo.encrypt = false;

  if (!esp_now_is_peer_exist(masterAddress)) {
    Serial.println("Voeg master toe als peer...");
    if (esp_now_add_peer(&masterInfo) == ESP_OK) {
      Serial.println("✅ Master toegevoegd als peer.");
    } else {
      Serial.println("❌ Fout bij toevoegen van master als peer.");
    }
  } else {
    Serial.println("Master al gekend als peer.");
  }

  pinMode(hallSensorTiltdropClosed, INPUT);
  pinMode(hallSensorTiltdropOpen, INPUT);
  pinMode(hallSensorOnTiltdrop, INPUT);
  pinMode(hallSensorOffTiltdrop, INPUT);

  tiltTrackStepper.setSpeed(10);

  Serial.println("SLAVE klaar voor gebruik.\n");
}

void loop() {
  char commandBuffer[100];

if (receiveCommand(commandBuffer, sizeof(commandBuffer))) {
  Serial.print("Ontvangen commando: ");
  Serial.println(commandBuffer);

  Serial.print("Vergelijk met IS_TILTDROP_CLOSED... ");
  if (strcmp(commandBuffer, "IS_TILTDROP_CLOSED") == 0) {
    Serial.println("MATCH!");
    tiltTrackStepper.setSpeed(10); // <-- nodig
    bool closed = IsTiltdropClosed();
    if (closed) sendLog("TILTDROP_CLOSED");
    else        sendLog("TILTDROP_OPEN");
  } else {
    Serial.println("GEEN MATCH!");
  }
}


  // Delay optioneel om CPU wat ademruimte te geven
  delay(10);
}

bool IsTiltdropClosed() {
  Serial.println("[Check] IsTiltdropClosed() gestart");

  int sensorVal = digitalRead(hallSensorTiltdropClosed);
  Serial.print("Sensorwaarde hallSensorTiltdropClosed: ");
  Serial.println(sensorVal);

  if (sensorVal == LOW) {
    sendLog("Tiltdrop CLOSED, ready.");
    Serial.println("✅ Tiltdrop is al gesloten.");
    return true;
  }

  sendLog("Tiltdrop NOT CLOSED! Attempting to close.");
  Serial.println("❗ Tiltdrop is niet gesloten. Beweeg motor...");

  for (int i = 0; i < 2048; i++) {
    tiltTrackStepper.step(1);

    if (i % 100 == 0) {
      Serial.print("... stappen: "); Serial.println(i);
    }

    if (digitalRead(hallSensorTiltdropClosed) == LOW) {
      tiltTrackStepper.step(5);
      sendLog("Tiltdrop CLOSED after movement.");
      Serial.println("✅ Gesloten na beweging.");
      return true;
    }
  }

  sendLog("ERROR: Could not close Tiltdrop.");
  Serial.println("❌ ERROR: Kon Tiltdrop niet sluiten na 2048 stappen.");
  return false;
}

void sendLog(const char* message) {
  struct_message msg;
  strncpy(msg.text, message, sizeof(msg.text) - 1);
  msg.text[sizeof(msg.text) - 1] = '\0';

  Serial.print("[SEND LOG] Verstuur naar master: ");
  Serial.print(message);

  esp_err_t result = esp_now_send(masterAddress, (uint8_t*)&msg, sizeof(msg));

  Serial.print(" -> ");
  if (result == ESP_OK) Serial.println("OK");
  else {
    Serial.print("FOUT (");
    Serial.print(result);
    Serial.println(")");
  }
}

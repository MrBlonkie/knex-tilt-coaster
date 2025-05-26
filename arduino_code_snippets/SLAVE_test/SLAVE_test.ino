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
  struct_message incoming;
  memcpy(&incoming, data, sizeof(incoming));

  // PING-pong afhandeling
  if (strcmp(incoming.text, "PING_TILTDROP") == 0) {
    sendPong("PONG_TILTDROP");
    return;  // Geen normale commandoprocessing doen
  }

  // Normaal commando opslaan
  memcpy(&incomingCommand, &incoming, sizeof(incomingCommand));
  newCommandAvailable = true;
}


void sendPong(const char* responseText) {
  struct_message response;
  strncpy(response.text, responseText, sizeof(response.text) - 1);
  response.text[sizeof(response.text) - 1] = '\0';

  esp_now_send(masterAddress, (uint8_t*)&response, sizeof(response));
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
    Serial.println("ESP-NOW init mislukt!");
    return;
  } else {
    Serial.println("ESP-NOW init geslaagd.");
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t masterInfo = {};
  memcpy(masterInfo.peer_addr, masterAddress, 6);
  masterInfo.channel = 0;
  masterInfo.encrypt = false;

  if (!esp_now_is_peer_exist(masterAddress)) {
    Serial.println("Voeg master toe als peer...");
    if (esp_now_add_peer(&masterInfo) == ESP_OK) {
      Serial.println("Master toegevoegd als peer.");
    } else {
      Serial.println("Fout bij toevoegen van master als peer.");
    }
  } else {
    Serial.println("Master al gekend als peer.");
  }

  pinMode(hallSensorTiltdropClosed, INPUT);
  pinMode(hallSensorTiltdropOpen, INPUT);
  pinMode(hallSensorOnTiltdrop, INPUT);
  pinMode(hallSensorOffTiltdrop, INPUT);

  tiltTrackStepper.setSpeed(10);
  releaseServo.attach(SERVO_PIN);

  Serial.println("SLAVE klaar voor gebruik.\n");
}

void loop() {

  char commandBuffer[100];

  if (receiveCommand(commandBuffer, sizeof(commandBuffer))) {
    Serial.print("Ontvangen commando: ");
    Serial.println(commandBuffer);

    if (strcmp(commandBuffer, "IS_TILTDROP_CLOSED") == 0) {
      bool closed = IsTiltdropClosed();
      if (closed) sendLog("TILTDROP_CLOSED");
      else        sendLog("TILTDROP_OPEN");
    } 

    if(strcmp(commandBuffer, "IS_COASTER_ON_TILTDROP") == 0) {
      EnterTiltdrop();
      sendLog("COASTER_ON_TILTDROP");
    }

    if(strcmp(commandBuffer, "DROP") == 0) {
      bool drop = Tiltdrop();
      if(drop){
        DropCoaster();
        sendLog("COASTER_DROPPED");
      } else {
        sendLog("NO_DROP");
      }     
    }

  }

  delay(10);

}

bool IsTiltdropClosed() {
  Serial.println("[Check] IsTiltdropClosed() gestart");

  Serial.println("resetting servo");
  for(int posDegrees = 90; posDegrees >= 0; posDegrees--) {
    releaseServo.write(posDegrees);
    delay(20);
  }
  Serial.println("servo reset success");

  int sensorVal = digitalRead(hallSensorTiltdropClosed);
  Serial.print("Sensorwaarde hallSensorTiltdropClosed: ");
  Serial.println(sensorVal);

  if (sensorVal == LOW) {
    sendLog("Tiltdrop CLOSED, ready.");
    Serial.println("Tiltdrop is al gesloten.");
    return true;
  }

  sendLog("Tiltdrop NOT CLOSED! Attempting to close.");
  Serial.println("Tiltdrop is niet gesloten. Beweeg motor...");

  for (int i = 0; i < 2048; i++) {
    tiltTrackStepper.step(1);

    if (digitalRead(hallSensorTiltdropClosed) == LOW) {
      tiltTrackStepper.step(10);
      sendLog("Tiltdrop CLOSED after movement.");
      Serial.println("Gesloten na beweging.");
      return true;
    }
  }

  sendLog("ERROR: Could not close Tiltdrop.");
  Serial.println("ERROR: Kon Tiltdrop niet sluiten na 2048 stappen.");
  return false;
}


void EnterTiltdrop()
{
  while (digitalRead(hallSensorOnTiltdrop) == HIGH) {
    Serial.println("coaster NOT on TILTDROP");
  }

  delay(100);
  Serial.println("coaster ON TILTDROP");
  delay(100);

}

bool Tiltdrop()
{

  tiltTrackStepper.setSpeed(5);
  delay(50);
  
  for (int i = 0; i < 2048; i++) {
    tiltTrackStepper.step(-1);

    if (digitalRead(hallSensorTiltdropOpen) == LOW) {
      Serial.println("Tiltdrop detected, doing extra steps to make sure it's open");
      tiltTrackStepper.step(-30);
      Serial.println("Tiltdrop OPEN, ready for drop");
      return true;
    }
  }

  Serial.println("ERROR: Failed to open Tiltdrop within step limit");
  return false;

}

void DropCoaster()
{
  for(int posDegrees = 0; posDegrees <= 90; posDegrees++) {
    releaseServo.write(posDegrees);
    delay(10);
  }

  delay(1000);
  Serial.println("Coaster released");
  delay(2000);

  for(int posDegrees = 90; posDegrees >= 0; posDegrees--) {
    releaseServo.write(posDegrees);
    delay(20);
  }

  Serial.println("Servo gesloten.");
  delay(100);

  Serial.println("Track resetten...");
  tiltTrackStepper.setSpeed(12);
  delay(500);
  tiltTrackStepper.step(stepsTiltTrack);
  delay(500);

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

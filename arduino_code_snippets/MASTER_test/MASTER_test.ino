#include <WiFi.h>
#include <esp_now.h>
#include "Message.h"

uint8_t tiltdropSlave[] = {0xF8, 0xB3, 0xB7, 0x33, 0x23, 0x00};  // Vervang door *jouw* SLAVE MAC
uint8_t stationSlave[] = {0xF8, 0XB3, 0xB7, 0x33, 0x41, 0xFC};

volatile bool newDataAvailable = false;
struct_message latestIncoming;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&latestIncoming, data, sizeof(latestIncoming));
  newDataAvailable = true;  // Flag dat er nieuwe data is
}

bool waitForResponse(const char* expectedResponse, unsigned long timeout = 3000) {
  unsigned long start = millis();
  while (!newDataAvailable && millis() - start < timeout) {
    delay(50); // minimal CPU-wait
  }

  if (!newDataAvailable) return false;

  newDataAvailable = false;

  return strcmp(latestIncoming.text, expectedResponse) == 0;
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.print("Master MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init mislukt!");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  //tiltdropSlave toevoegen
  esp_now_peer_info_t slaveInfo = {};
  memcpy(slaveInfo.peer_addr, tiltdropSlave, 6);
  slaveInfo.channel = 0;
  slaveInfo.encrypt = false;

  if (!esp_now_is_peer_exist(tiltdropSlave)) {
    esp_now_add_peer(&slaveInfo);
  }

  //stationSlave toevoegen
  esp_now_peer_info_t stationPeerInfo = {};
  memcpy(stationPeerInfo.peer_addr, stationSlave, 6);
  stationPeerInfo.channel = 0;
  stationPeerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(stationSlave)) {
    esp_now_add_peer(&stationPeerInfo);
  }

  sendCommand("PING_TILTDROP", tiltdropSlave);
  if (waitForResponse("PONG_TILTDROP")) {
    Serial.println("Tiltdrop slave online");
  } else {
    Serial.println("Tiltdrop slave NIET bereikbaar!");
  }

  sendCommand("PING_STATION", stationSlave);
  if (waitForResponse("PONG_STATION")) {
    Serial.println("Station slave online");
  } else {
    Serial.println("Station slave NIET bereikbaar!");
  }



  Serial.println("MASTER klaar.");

  delay(1000);  // even wachten voor zekerheid

}

void loop() {
  delay(5000);
  WaitForInput('S', "Starting........");

  sendCommand("DISPATCH", stationSlave);
  bool isCoasterDispatched = DispatchResponse();

  bool tiltdropStatus = false;
  while(!tiltdropStatus){
  sendCommand("IS_TILTDROP_CLOSED", tiltdropSlave);
  tiltdropStatus = TiltdropResponse(3);
  }
  

  StartAndStopLifthillMotor();

 


  sendCommand("DROP", tiltdropSlave);
  DropResponse();


}




void sendCommand(const char* commandText, const uint8_t* destinationMAC) {
  struct_message msg;
  strncpy(msg.text, commandText, sizeof(msg.text) - 1);
  msg.text[sizeof(msg.text) - 1] = '\0'; // safety null-terminate

  esp_err_t result = esp_now_send(destinationMAC, (uint8_t*)&msg, sizeof(msg));

  Serial.print("Stuur commando: ");
  Serial.print(commandText);
  Serial.print(" -> ");

  if (result == ESP_OK) {
    Serial.println("SUCCES");
  } else {
    Serial.print("FOUT: ");
    Serial.println(result);
  }
}


void StartAndStopLifthillMotor(){
  sendCommand("LIFTHILL_MOTOR_ON", stationSlave);
  LifthillResponse();
  
  sendCommand("IS_COASTER_ON_TILTDROP", tiltdropSlave);
  if(CoasterOnTiltdropResponse()){
    sendCommand("LIFTHILL_MOTOR_OFF", stationSlave);
    LifthillResponse();
  }
}

bool TiltdropResponse(int maxRetries) {
  int retries = 0;

  while (retries < maxRetries) {
    // Wait for new data, timeout after 3 seconds
    unsigned long start = millis();
    while (!newDataAvailable && millis() - start < 3000) {
      delay(50);  // small delay to avoid hogging CPU
    }
    if (!newDataAvailable) {
      Serial.println("Timeout waiting for response.");
      return false;
    }

    Serial.print("Verwerkt ontvangen data: ");
    Serial.println(latestIncoming.text);

    if (strcmp(latestIncoming.text, "TILTDROP_CLOSED") == 0) {
      newDataAvailable = false;
      return true;
    }
    else if (strcmp(latestIncoming.text, "TILTDROP_OPEN") == 0) {
      newDataAvailable = false;
      Serial.println("Tiltdrop open, retrying...");
      sendCommand("IS_TILTDROP_CLOSED", tiltdropSlave);
      retries++;
      delay(1000);  // brief wait before next try
      continue;
    }
    else {
      Serial.println("Unexpected response:");
      Serial.println(latestIncoming.text);
      newDataAvailable = false;
      return false;
    }
  }

  Serial.println("Max retries reached without TILTDROP_CLOSED.");
  return false;
}

bool DispatchResponse() {
  WaitOnData();

  if (strcmp(latestIncoming.text, "COASTER_DISPATCHED") == 0) {
    Serial.println("coaster dispatched, check for bottomsensor");
    newDataAvailable = false;
    return true;
  }
}

bool LifthillResponse() {

  WaitOnData();

  if (strcmp(latestIncoming.text, "MOTOR_RUNNING") == 0) {
    Serial.println("station motor running");
    newDataAvailable = false;
    return true;
  } else if(strcmp(latestIncoming.text, "MOTOR_STOPPED") == 0){
    Serial.println("station motor stopped");
    newDataAvailable = false;
    return false;
  }else {
    Serial.println("Something went wrong. Please check logs and reset.");
    Serial.println(latestIncoming.text);
    newDataAvailable = false;
    return false;
  }
}

bool CoasterOnTiltdropResponse() {
  while(!newDataAvailable){
    Serial.println("wachten op respone");
    delay(2000);
  }

  Serial.print("Verwerkt ontvangen data: ");
  Serial.println(latestIncoming.text);

  if (strcmp(latestIncoming.text, "COASTER_ON_TILTDROP") == 0) {
    // bv. update statusvariabele of trigger actie
    Serial.println("Coaster is on tiltdrop, ready for drop");
    newDataAvailable = false;
    return true;
  } else {
    Serial.println("Something went wrong. Please check logs and reset.");
    Serial.println(latestIncoming.text);
    newDataAvailable = false;
    return false;
    }
  }

bool DropResponse() {
while(!newDataAvailable){
    Serial.println("wachten op response");
    delay(1000);
  }

  Serial.print("Verwerkt ontvangen data: ");
  Serial.println(latestIncoming.text);

  if (strcmp(latestIncoming.text, "COASTER_DROPPED") == 0) {
    // bv. update statusvariabele of trigger actie
    Serial.println("Coaster dropped, ready for next sequence");
    newDataAvailable = false;
    return true;
  } else {
    Serial.println("Something went wrong. Please check logs and reset.");
    Serial.println(latestIncoming.text);
    newDataAvailable = false;
    return false;
    }
  }

void WaitForInput(char expectedChar, String succesMessage)
{
  expectedChar = toupper(expectedChar);

  while (true) {
    if (Serial.available() > 0) {
      char input = Serial.read();
      input = toupper(input);

      if (input == expectedChar) {
        Serial.println(succesMessage);
        break;
      }
    }
  }
}

void WaitOnData() {
  while(!newDataAvailable){
    Serial.println("wachten op response");
    delay(1000);
  }

  Serial.print("Verwerkt ontvangen data: ");
  Serial.println(latestIncoming.text);
}
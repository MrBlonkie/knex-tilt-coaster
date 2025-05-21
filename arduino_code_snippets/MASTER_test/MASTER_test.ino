#include <WiFi.h>
#include <esp_now.h>
#include "Message.h"

uint8_t slaveAddress[] = {0xF8, 0xB3, 0xB7, 0x33, 0x23, 0x00};  // Vervang door *jouw* SLAVE MAC

volatile bool newDataAvailable = false;
struct_message latestIncoming;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&latestIncoming, data, sizeof(latestIncoming));
  newDataAvailable = true;  // Flag dat er nieuwe data is
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

  esp_now_peer_info_t slaveInfo = {};
  memcpy(slaveInfo.peer_addr, slaveAddress, 6);
  slaveInfo.channel = 0;
  slaveInfo.encrypt = false;

  if (!esp_now_is_peer_exist(slaveAddress)) {
    esp_now_add_peer(&slaveInfo);
  }

  Serial.println("MASTER klaar.");

  delay(1000);  // even wachten voor zekerheid

  Serial.println("PING gestuurd naar SLAVE...");
}

void loop() {
  delay(1000);

  sendCommand("IS_TILTDROP_CLOSED", slaveAddress);
  TiltdropResponse();

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

void TiltdropResponse() {

  while(!newDataAvailable){
    Serial.println("wachten op respone");
    delay(500);
  }

  Serial.print("Verwerkt ontvangen data: ");
  Serial.println(latestIncoming.text);

  if (strcmp(latestIncoming.text, "TILTDROP_CLOSED") == 0) {
    // bv. update statusvariabele of trigger actie
    Serial.println("TODO: lifthill motor aansturen");
  }
  else if (strcmp(latestIncoming.text, "TILTDROP_OPEN") == 0) {
    // andere actie
    Serial.println("TODO: wachten en later opnieuw checken");
  } else {
    Serial.println("Something went wrong. Please check logs and reset.");
  }
  
  // Data verwerkt, reset flag
  newDataAvailable = false;
}

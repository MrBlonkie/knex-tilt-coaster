#include <esp_now.h>
#include <WiFi.h>
#include "Message.h"

uint8_t slaveAddress[] = {0xF8, 0xB3, 0xB7, 0x33, 0x23, 0x00}; // Pas dit aan
struct_message outgoingMessage;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  struct_message msg;
  memcpy(&msg, data, sizeof(msg));
  Serial.print("[SLAVE] ");
  Serial.println(msg.text);
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  delay(1000);
  strcpy(outgoingMessage.text, "START");
  esp_now_send(slaveAddress, (uint8_t *) &outgoingMessage, sizeof(outgoingMessage));
  Serial.println("START command sent to SLAVE.");
}

void loop() {
  // hier kun je in de toekomst knoppen, bluetooth triggers, e.d. integreren
}

#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "wieditleestisZOT";
const char* password = "jemama123";

WebServer server(80);
const int LED_PIN = 2;

bool ledState = false;

void handleNotFound(){ server.send(404, "text/plain", "Not found"); }

void handleLedOn(){
  digitalWrite(LED_PIN, HIGH);
  ledState = true;
  server.send(200, "application/json", "{\"status\":\"on\"}");
}

void handleLedOff(){
  digitalWrite(LED_PIN, LOW);
  ledState = false;
  server.send(200, "application/json", "{\"status\":\"off\"}");
}

void handleLedStatus() {
  String json = "{\"onboardLED\":" + String(ledState ? "true" : "false") + "}";
  server.send(200, "application/json", json);
  Serial.println("LED status opgevraagd");
}

void setup(){
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  server.on("/control/on", HTTP_POST, handleLedOn);
  server.on("/control/off", HTTP_POST, handleLedOff);
  server.on("/control/status", HTTP_GET, handleLedStatus);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server gestart");
}

void loop(){
  server.handleClient();
}

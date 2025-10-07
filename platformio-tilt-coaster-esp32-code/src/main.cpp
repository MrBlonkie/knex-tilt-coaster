#include <WiFi.h>
#include <WebServer.h>
#include <Stepper.h>
#include <Arduino.h>

// motors
#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23

#define IN5 13
#define IN6 12
#define IN7 14
#define IN8 27

const int stepsPerRevolution = 2048;

Stepper stationStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);
Stepper lifthillStepper(stepsPerRevolution, IN5, IN7, IN6, IN8);

int lifthillSteps = 0;
int stationSteps = 0;


//hall sensoren
#define hallSensorEnterStation     34
#define hallSensorStartPosition    32
#define hallSensorExitStation      35
#define hallSensorBottomLifthill   33

bool hallSensorEnterStationState = false;
bool hallSensorStartPositionState = false;
bool hallSensorExitStationState = false;
bool hallSensorBottomLifthillState = false;


// server stuff
const char* ssid = "wieditleestisZOT";
const char* password = "jemama123";

WebServer server(80);
const int LED_PIN = 2;

bool ledState = false;
bool stationMotorState = false;
bool lifthillMotorState = false;

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// LED handlers
void handleLedOn() {
  digitalWrite(LED_PIN, HIGH);
  ledState = true;
  server.send(200, "application/json", "{\"led\":\"on\"}");
}

void handleLedOff() {
  digitalWrite(LED_PIN, LOW);
  ledState = false;
  server.send(200, "application/json", "{\"led\":\"off\"}");
}

// Station motor handlers
void handleStationMotorOn() {
  stationMotorState = true;
  server.send(200, "application/json", "{\"stationMotor\":\"on\"}");
}

void handleStationMotorOff() {
  stationMotorState = false;
  server.send(200, "application/json", "{\"stationMotor\":\"off\"}");
}

// Lifthill motor handlers
void handleLifthillMotorOn() {
  lifthillMotorState = true;
  server.send(200, "application/json", "{\"lifthillMotor\":\"on\"}");
}

void handleLifthillMotorOff() {
  lifthillMotorState = false;
  server.send(200, "application/json", "{\"lifthillMotor\":\"off\"}");
}

// Motor stepping
void handleStationMotorStep() {
  if (stationMotorState) {
    stationStepper.step(20); // 20 stappen per loop
  }
}

void handleLifthillMotorStep() {
  if (lifthillMotorState) {
    lifthillStepper.step(-20); // 20 stappen per loop
  }
}

//auto-control logic
bool coasterDispatched = false;

void handleDispatch() {
  coasterDispatched = true;
  server.send(200, "application/json", "{\"dispatch\":\"started\"}");
  hallSensorEnterStationState = false;
  hallSensorStartPositionState = false;
  hallSensorBottomLifthillState = false;
  hallSensorExitStationState = false;
}



//Auto Control Status handler
void handleAutoControlStatus() {
  String json = "{";
  json += "\"hallSensorExitStation\":"; json += (hallSensorExitStationState ? "true" : "false");
  json += "\"hallSensorBottomLifthill\":"; json += (hallSensorBottomLifthillState ? "true" : "false");
  json += "\"hallSensorEnterStation\":"; json += (hallSensorEnterStationState ? "true" : "false");
  json += "\"hallSensorStartPosition\":"; json += (hallSensorStartPositionState ? "true" : "false");
  json += "\"coasterDispatched\":"; json += (coasterDispatched ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
  Serial.println("Control status opgevraagd: " + json);
}


// Manual Control Status handler
void handleManualControlStatus() {
  String json = "{";
  json += "\"onboardLED\":"; json += (ledState ? "true" : "false");
  json += ",\"stationMotor\":"; json += (stationMotorState ? "true" : "false");
  json += ",\"lifthillMotor\":"; json += (lifthillMotorState ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
  Serial.println("Control status opgevraagd: " + json);
}


void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(hallSensorEnterStation, INPUT);
  pinMode(hallSensorStartPosition, INPUT);
  pinMode(hallSensorExitStation, INPUT);
  pinMode(hallSensorBottomLifthill, INPUT);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // routes
  //manual control
  server.on("/led/on", HTTP_POST, handleLedOn);
  server.on("/led/off", HTTP_POST, handleLedOff);
  
  server.on("/motor/station/on", HTTP_POST, handleStationMotorOn);
  server.on("/motor/station/off", HTTP_POST, handleStationMotorOff);

  server.on("/motor/lifthill/on", HTTP_POST, handleLifthillMotorOn);
  server.on("/motor/lifthill/off", HTTP_POST, handleLifthillMotorOff);

  server.on("/manual-control/status", HTTP_GET, handleManualControlStatus);

  //auto control
  server.on("/dispatch/go", HTTP_POST, handleDispatch);

  server.on("/auto-control/status", HTTP_GET, handleAutoControlStatus);
  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server gestart");
  Serial.println("platformio test geslaagd");

  stationStepper.setSpeed(10);
  lifthillStepper.setSpeed(10);



}

void loop() {
  server.handleClient();
  handleStationMotorStep();
  handleLifthillMotorStep();

  if (coasterDispatched) {
    stationStepper.step(1);
    if (digitalRead(hallSensorExitStation) == LOW && stationSteps < 200) {
      hallSensorExitStationState = true;
      stationSteps++;
      Serial.println("Coaster Dispatched!");
    }
  }
}

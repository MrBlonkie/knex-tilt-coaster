#include <WiFi.h>
#include <esp_now.h>
#include <Stepper.h>
#include <ESP32Servo.h>
#include "Message.h"

#define hallSensorEnterStation     26
#define hallSensorStartPosition    25
#define hallSensorExitStation      33
bool hallSensorExitStationState = false;
#define hallSensorBottomLifthill   32
#define hallSensorTopLifthill      34

#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23

#define IN5 13
#define IN6 12
#define IN7 14
#define IN8 27

const int stepsPerRevolution = 2048;
const int extraStationSteps = 50;

TaskHandle_t motorTaskHandle = NULL;
bool lifthillMotorRunning = false;

Stepper lifthillStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);
Stepper stationStepper(stepsPerRevolution, IN5, IN7, IN6, IN8);

uint8_t masterAddress[] = {0x14, 0x2B, 0x2F, 0xC9, 0x24, 0xCC}; // MASTER MAC-address

volatile bool newCommandAvailable = false;
struct_message incomingCommand;

// Callback voor inkomende ESP-NOW data
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  struct_message incoming;
  memcpy(&incoming, data, sizeof(incoming));

  Serial.print("Ontvangen van master: ");
  Serial.println(incoming.text);

  if (strcmp(incoming.text, "PING_STATION") == 0) {
    sendPong("PONG_STATION");
  } else {
    memcpy(&incomingCommand, &incoming, sizeof(incoming));
    newCommandAvailable = true;
  }
}

// Stuurt een "pong" terug naar de master
void sendPong(const char* responseText) {
  struct_message msg;
  strncpy(msg.text, responseText, sizeof(msg.text) - 1);
  msg.text[sizeof(msg.text) - 1] = '\0';

  Serial.print("Stuur PONG: ");
  Serial.println(msg.text);

  esp_now_send(masterAddress, (uint8_t*)&msg, sizeof(msg));
}

// Haalt binnengekomen commando uit de buffer
bool receiveCommand(char* buffer, size_t bufferSize) {
  if (!newCommandAvailable) return false;

  strncpy(buffer, incomingCommand.text, bufferSize - 1);
  buffer[bufferSize - 1] = '\0';
  newCommandAvailable = false;
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== STATION SLAVE SETUP ===");

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

  pinMode(hallSensorEnterStation, INPUT);
  pinMode(hallSensorStartPosition, INPUT);
  pinMode(hallSensorExitStation, INPUT);
  pinMode(hallSensorBottomLifthill, INPUT);
  pinMode(hallSensorTopLifthill, INPUT);

  lifthillStepper.setSpeed(10);
  stationStepper.setSpeed(12);

  Serial.println("SLAVE klaar voor gebruik.\n");
}

void loop() {
  char commandBuffer[100];

  if (receiveCommand(commandBuffer, sizeof(commandBuffer))) {
    Serial.print("Ontvangen commando: ");
    Serial.println(commandBuffer);

    if (strcmp(commandBuffer, "LIFTHILL_MOTOR_ON") == 0) {
      StartMotor();
      sendLog("MOTOR_RUNNING");
    }

    if (strcmp(commandBuffer, "LIFTHILL_MOTOR_OFF") == 0) {
      StopMotor(); 
      sendLog("MOTOR_STOPPED");
    }

    if (strcmp(commandBuffer, "DISPATCH") == 0) {
      DispatchCoaster(); 
      sendLog("COASTER_DISPATCHED");
    }


  }

  delay(10);
}

void motorTask(void* parameter) {
  Serial.println("[TASK] Steppermotor draait...");

  lifthillStepper.setSpeed(10);  // of pas aan naar gewenste snelheid

  while (lifthillMotorRunning) {
    lifthillStepper.step(-1);  // of -1 voor andere richting
    vTaskDelay(2 / portTICK_PERIOD_MS); // 2ms delay tussen stappen
  }

  motorTaskHandle = NULL;

  Serial.println("[TASK] motorTask eindigt.");
  vTaskDelete(NULL); // Beëindig deze taak netjes
}

void StartMotor() {
  if (lifthillMotorRunning) {
    Serial.println("Motor draait al.");
    return;
  }

  Serial.println("Motor STARTEN via FreeRTOS task...");
  lifthillMotorRunning = true;

  xTaskCreate(
    motorTask,         // Functie
    "MotorTask",       // Naam
    2048,              // Stack size (bytes)
    NULL,              // Parameter
    1,                 // Prioriteit
    &motorTaskHandle   // Taakhandle bewaren
  );
}

void StopMotor() {
  if (!lifthillMotorRunning) {
    Serial.println("Motor is al gestopt.");
    return;
  }

  Serial.println("STOP signaal ontvangen: motor stoppen...");
  lifthillMotorRunning = false;

  // Laat de taak zichzelf stoppen via vTaskDelete(NULL) in de loop
  // of forceer hem hierna als je zeker wil zijn:
  if (motorTaskHandle != NULL) {
    vTaskDelete(motorTaskHandle);
    motorTaskHandle = NULL;
  }
}

bool DispatchCoaster() {
  while(digitalRead(hallSensorExitStation) == HIGH) {
    stationStepper.step(1); 
  }
  Serial.println("Coaster Dispatched!");
  hallSensorExitStationState = true;


  return true;
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

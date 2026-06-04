#include <AccelStepper.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include "../shared/env.h"

// === WiFi & MQTT ===
WiFiClient espClient;
PubSubClient client(espClient);
const char *deviceName = "switchtrack";
String clientId;
const char *connectivityTopic = "rollercoaster/switchtrack/status";

// === Heartbeat Config ===
unsigned long lastHeartbeat = 0;
const long heartbeatInterval = 2500;

// === Motor Config ===
#define ROTATING_IN1 19
#define ROTATING_IN2 21
#define ROTATING_IN3 22
#define ROTATING_IN4 23

#define TILTING_IN1 14
#define TILTING_IN2 13
#define TILTING_IN3 15
#define TILTING_IN4 18

// === Servo config ===
#define SERVO_PIN 33
Servo releaseServo;
bool releaseswitchMotorState = false;

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

// === Sensor config ===
#define hallSensorStationConnect 27
#define hallSensorBrakeConnect 26
#define hallSensorMiddle 25
#define hallSensorOnSwitchtrack 32

AccelStepper rotatingStepper(AccelStepper::FULL4WIRE, ROTATING_IN1, ROTATING_IN3, ROTATING_IN2, ROTATING_IN4);
AccelStepper tiltingStepper(AccelStepper::FULL4WIRE, TILTING_IN1, TILTING_IN3, TILTING_IN2, TILTING_IN4);

bool hallSensorStationConnectState = false;
bool hallSensorMiddleState = false;
bool hallSensorBrakeConnectState = false;
bool hallSensorOnSwitchtrackState = false;

bool manualMode = false;
bool coasterDispatched = false;
String manualRotateTarget = "";
float targetSpeed = 0;
bool isSwitchtrackAtBrakes = false;

bool trainOnLayout = false;
String latestEventMsg = "";

bool isSwitchtrackOccupied = false;
bool stabilizationStarted = false;
bool isNextBlockFree = false;
bool releaseTriggered = false;
bool resetStarted = false;

// === Beweging richting BRAKES ===
const int   BRAKES_TILT_FREE_STEPS     = 70;   // Initiële tilt om los te komen van de station guiding rail
const int   BRAKES_ROTATE_PARTIAL_STEPS = 800;  // Vaste rotatiestappen vóór mid-tilt
const int   BRAKES_TILT_MID_STEPS      = 100;   // Extra tiltstappen na gedeeltelijke rotatie
const float BRAKES_ROTATE_SPEED        = 400.0; // Rotatiesnelheid (steps/sec)
const float BRAKES_TILT_SPEED          = 250.0; // Tiltsnelheid voor free en mid
const int   BRAKES_ROTATE_EXTRA_STEPS  = 50;    // Extra rotatiestappen na brakes sensor
const int   BRAKES_TILT_SEAT_STEPS     = 0;   // Tiltstappen voor zachte plaatsing in brakes guiding rail
const float BRAKES_TILT_SEAT_SPEED     = 150.0; // Langzame snelheid voor zachte plaatsing

// === Beweging richting STATION ===
const int   STATION_TILT_FREE_STEPS     = 50;   // Initiële tilt om los te komen van de brakes guiding rail
const int   STATION_ROTATE_PARTIAL_STEPS = 900;  // Vaste rotatiestappen vóór mid-tilt
const int   STATION_TILT_MID_STEPS      = 70;   // Extra tiltstappen na gedeeltelijke rotatie
const float STATION_ROTATE_SPEED        = 400.0; // Rotatiesnelheid (steps/sec)
const float STATION_TILT_SPEED          = 250.0; // Tiltsnelheid voor free en mid
const int   STATION_ROTATE_EXTRA_STEPS  = 50;     // Extra r  otatiestappen na station sensor
const int   STATION_TILT_SEAT_STEPS     = 0;   // Tiltstappen voor zachte plaatsing in station guiding rail
const float STATION_TILT_SEAT_SPEED     = 150.0; // Langzame snelheid voor zachte plaatsing

bool trainInStationFlag = false;
bool isSwitchtrackMoving = false;

bool onSwitchtrackFlag = false;
bool resetSwitchtrackFlag = false;
bool switchtrackResetted = false;
bool switchtrackFreeFlag = false;
bool switchtrackSafetyFlag = false;

unsigned long stateStartTime = 0; // Timer voor in de state machine
bool timerActive = false;         // Om te checken of we aan het wachten zijn

enum MovementPhase
{
  PHASE_IDLE,
  PHASE_TILT_FREE,        // Initiële tilt om los te komen van huidige guiding rail
  PHASE_ROTATE_PARTIAL,   // Vaste rotatiestappen vóór mid-tilt
  PHASE_TILT_MID,         // Mid-tiltstappen na gedeeltelijke rotatie
  PHASE_ROTATE,           // Roteer tot aankomstsensor
  PHASE_ROTATE_EXTRA,     // Extra rotatiestappen na aankomstsensor
  PHASE_TILT_SEAT,        // Zachte plaatsing in nieuwe guiding rail
};

void handleMovement()
{
  static MovementPhase phase = PHASE_IDLE;
  static int tiltDir = 0;

  // Gecachete waarden per beweging (ingesteld bij START)
  static int   s_tiltFreeSteps, s_rotatePartialSteps, s_tiltMidSteps;
  static int   s_rotateExtraSteps, s_tiltSeatSteps;
  static float s_rotateSpeed, s_tiltSpeed, s_tiltSeatSpeed;

  // Emergency stop
  if (targetSpeed == 0 && phase != PHASE_IDLE)
  {
    rotatingStepper.stop();
    tiltingStepper.stop();
    phase = PHASE_IDLE;
    tiltingStepper.stop();
    phase = PHASE_IDLE;
    isSwitchtrackMoving = false;
    return;
  }

  if (targetSpeed == 0 && phase == PHASE_IDLE)
  {
    isSwitchtrackMoving = false;
    return;
  }

  // --- START: cache richting-specifieke config ---
  if (phase == PHASE_IDLE)
  {
    tiltDir = (targetSpeed > 0) ? 1 : -1;
    isSwitchtrackMoving = true;

    if (tiltDir < 0)
    {
      s_tiltFreeSteps      = BRAKES_TILT_FREE_STEPS;
      s_rotatePartialSteps = BRAKES_ROTATE_PARTIAL_STEPS;
      s_tiltMidSteps       = BRAKES_TILT_MID_STEPS;
      s_rotateSpeed        = BRAKES_ROTATE_SPEED;
      s_tiltSpeed          = BRAKES_TILT_SPEED;
      s_rotateExtraSteps   = BRAKES_ROTATE_EXTRA_STEPS;
      s_tiltSeatSteps      = BRAKES_TILT_SEAT_STEPS;
      s_tiltSeatSpeed      = BRAKES_TILT_SEAT_SPEED;
    }
    else
    {
      s_tiltFreeSteps      = STATION_TILT_FREE_STEPS;
      s_rotatePartialSteps = STATION_ROTATE_PARTIAL_STEPS;
      s_tiltMidSteps       = STATION_TILT_MID_STEPS;
      s_rotateSpeed        = STATION_ROTATE_SPEED;
      s_tiltSpeed          = STATION_TILT_SPEED;
      s_rotateExtraSteps   = STATION_ROTATE_EXTRA_STEPS;
      s_tiltSeatSteps      = STATION_TILT_SEAT_STEPS;
      s_tiltSeatSpeed      = STATION_TILT_SEAT_SPEED;
    }

    tiltingStepper.setMaxSpeed(s_tiltSpeed);
    tiltingStepper.setAcceleration(200);
    tiltingStepper.move(tiltDir * s_tiltFreeSteps);
    phase = PHASE_TILT_FREE;
    return;
  }

  // --- FASE 1: Initiële tilt ---
  if (phase == PHASE_TILT_FREE)
  {
    tiltingStepper.run();
    if (tiltingStepper.distanceToGo() == 0)
    {
      rotatingStepper.setMaxSpeed(s_rotateSpeed);
      rotatingStepper.setAcceleration(400);
      rotatingStepper.move(tiltDir * s_rotatePartialSteps);
      phase = PHASE_ROTATE_PARTIAL;
    }
    return;
  }

  // --- FASE 2: Gedeeltelijke rotatie ---
  if (phase == PHASE_ROTATE_PARTIAL)
  {
    rotatingStepper.run();
    if (rotatingStepper.distanceToGo() == 0)
    {
      tiltingStepper.setMaxSpeed(s_tiltSpeed);
      tiltingStepper.setAcceleration(200);
      tiltingStepper.move(tiltDir * s_tiltMidSteps);
      phase = PHASE_TILT_MID;
    }
    return;
  }

  // --- FASE 3: Mid-tilt ---
  if (phase == PHASE_TILT_MID)
  {
    tiltingStepper.run();
    if (tiltingStepper.distanceToGo() == 0)
    {
      rotatingStepper.setMaxSpeed(s_rotateSpeed);
      rotatingStepper.setSpeed(tiltDir * s_rotateSpeed);
      phase = PHASE_ROTATE;
    }
    return;
  }

  // --- FASE 4: Roteer tot aankomstsensor ---
  if (phase == PHASE_ROTATE)
  {
    bool arrived = (tiltDir > 0) ? hallSensorStationConnectState : hallSensorBrakeConnectState;

    if (arrived)
    {
      rotatingStepper.stop();
      rotatingStepper.setMaxSpeed(s_rotateSpeed);
      rotatingStepper.setAcceleration(400);
      rotatingStepper.move(tiltDir * s_rotateExtraSteps);
      phase = PHASE_ROTATE_EXTRA;
      return;
    }

    rotatingStepper.runSpeed();
    return;
  }

  // --- FASE 5: Extra rotatie na sensor ---
  if (phase == PHASE_ROTATE_EXTRA)
  {
    rotatingStepper.run();
    if (rotatingStepper.distanceToGo() == 0)
    {
      tiltingStepper.setMaxSpeed(s_tiltSeatSpeed);
      tiltingStepper.setAcceleration(100);
      tiltingStepper.move(tiltDir * s_tiltSeatSteps);
      phase = PHASE_TILT_SEAT;
    }
    return;
  }

  // --- FASE 6: Zachte plaatsing in guiding rail ---
  if (phase == PHASE_TILT_SEAT)
  {
    tiltingStepper.run();
    if (tiltingStepper.distanceToGo() == 0)
    {
      isSwitchtrackMoving = false;
      targetSpeed = 0;
      phase = PHASE_IDLE;
    }
    return;
  }
}

// === MQTT callback ===
void callback(char *topic, byte *payload, unsigned int length)
{
  String message;
  for (int i = 0; i < length; i++)
    message += (char)payload[i];

  if (String(topic) == "switchtrack/manual")
  {
    if (message == "on")
    {
      manualMode = true;
      Serial.println("Manual mode ENABLED");
    }
    else if (message == "off")
    {
      manualMode = false;
      coasterDispatched = false;
      // reset motors
      targetSpeed = -500;
      manualRotateTarget = "brakes";
      targetPos = 90;
      releaseswitchMotorState = false;
      Serial.println("Manual mode DISABLED");
    }
  }
  else if (manualMode)
  {
    if (String(topic) == "switchtrack/rotatemotor")
    {
      if (message == "on")
      {
        Serial.println("COMMAND: Rotate to Brakes");
        targetSpeed = -500;
        manualRotateTarget = "brakes";
      }
      if (message == "off")
      {
        Serial.println("COMMAND: Rotate to Station");
        targetSpeed = 500;
        manualRotateTarget = "station";
      }
    }
    else if (String(topic) == "switchtrack/releaseswitchtrackmotor")
    {
      if (message == "open")
      {
        Serial.println("COMMAND: Servo Open");
        targetPos = 0;
        releaseswitchMotorState = true;
      }
      if (message == "close")
      {
        Serial.println("COMMAND: Servo Close");
        targetPos = 90;
        releaseswitchMotorState = false;
      }
    }
  }

  else if (String(topic) == "rollercoaster/dispatch" && message == "go" && !manualMode)
  {
    coasterDispatched = true;
    Serial.println("recieved dipsatch go command");
  }
  else if (String(topic) == "rollercoaster/dispatch" && message == "stop")
  {
    coasterDispatched = false;
  }

  if (String(topic) == "rollercoaster/clear/switchtrack" && message == "clear")
  {
    switchtrackSafetyFlag = false;
    client.publish("rollercoaster/event", "cleared_switchtrack_safety_flag");
  }
  if (String(topic) == "rollercoaster/event" && message == "train_enters_station")
  {
    trainInStationFlag = true;
  }
  if (String(topic) == "rollercoaster/event" && message == "train_left_station")
  {
    trainInStationFlag = false;
  }
  // Block System Events
  if (String(topic) == "rollercoaster/block/event")
  {
    if (message == "station_free")
      isNextBlockFree = true;
    if (message == "station_occupied")
      isNextBlockFree = false;
  }

  if (String(topic) == "rollercoaster/estop" && message == "stop")
  {
    targetSpeed = 0;
    coasterDispatched = false;
    targetPos = 90;
    releaseswitchMotorState = false;
    client.publish("rollercoaster/event", "estop_switchtrack");
  }
}

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

// === Sensors update ===
void updateSensors()
{
  hallSensorBrakeConnectState = digitalRead(hallSensorBrakeConnect) == LOW;
  hallSensorMiddleState = digitalRead(hallSensorMiddle) == LOW;
  hallSensorStationConnectState = digitalRead(hallSensorStationConnect) == LOW;
  hallSensorOnSwitchtrackState = digitalRead(hallSensorOnSwitchtrack) == LOW;
}

// === Publish Status ===
String lastStatus = "";
void publishStatusIfChanged()
{
  String status = "{";

  status += "\"sensors\":{";
  status += "\"hallSensorBrakeConnectState\":" + String(hallSensorBrakeConnectState ? "true" : "false");
  status += ",\"hallSensorMiddleState\":" + String(hallSensorMiddleState ? "true" : "false");
  status += ",\"hallSensorStationConnectState\":" + String(hallSensorStationConnectState ? "true" : "false");
  status += ",\"hallSensorOnSwitchtrackState\":" + String(hallSensorOnSwitchtrackState ? "true" : "false");
  status += "},";

  status += "\"switchtrack\":{";
  status += "\"releaseswitchMotorState\":" + String(releaseswitchMotorState ? "true" : "false");
  status += ",\"isSwitchtrackMoving\":" + String(isSwitchtrackMoving ? "true" : "false");
  status += ",\"manualRotateTarget\":\"" + manualRotateTarget + "\"";
  status += "},";

  status += "\"mode\":{";
  status += "\"manualMode\":" + String(manualMode ? "true" : "false");
  status += "},";

  status += "\"blocks\":{";
  status += "\"isSwitchtrackOccupied\":" + String(isSwitchtrackOccupied ? "true" : "false");
  status += ",\"isNextBlockFree\":" + String(isNextBlockFree ? "true" : "false");
  status += "},";

  status += "\"flags\":{";
  status += "\"onSwitchtrackFlag\":" + String(onSwitchtrackFlag ? "true" : "false");
  status += ",\"resetSwitchtrackFlag\":" + String(resetSwitchtrackFlag ? "true" : "false");
  status += ",\"switchtrackResetted\":" + String(switchtrackResetted ? "true" : "false");
  status += ",\"switchtrackFreeFlag\":" + String(switchtrackFreeFlag ? "true" : "false");
  status += ",\"switchtrackSafetyFlag\":" + String(switchtrackSafetyFlag ? "true" : "false");
  status += ",\"trainInStationFlag\":" + String(trainInStationFlag ? "true" : "false");
  status += "}";

  status += "}";

  if (status != lastStatus)
  {
    client.publish("switchtrack/status", status.c_str(), true);
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

      client.subscribe("switchtrack/manual");
      client.subscribe("switchtrack/rotatemotor");
      client.subscribe("switchtrack/releaseswitchtrackmotor");
      client.subscribe("rollercoaster/control/auto");
      client.subscribe("rollercoaster/event");
      client.subscribe("rollercoaster/dispatch");

      client.subscribe("rollercoaster/block/event");
      client.subscribe("rollercoaster/clear/switchtrack");
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

void handleSwitchtrackBlockV2()
{

  unsigned long now = millis();

  if (hallSensorOnSwitchtrackState && !onSwitchtrackFlag && !isSwitchtrackOccupied && !switchtrackSafetyFlag)
  {
    if (!stabilizationStarted)
    {
      stateStartTime = now;
      stabilizationStarted = true;
      client.publish("rollercoaster/block/event", "switchtrack_occupied");
      client.publish("rollercoaster/event", "train_on_switchtrack");
      
      // CRUCIAL: Reset de stationsvlag hier, anders denkt de code dat de trein er al is!
      trainInStationFlag = false; 
    }

    if (stabilizationStarted && (now - stateStartTime >= 2000))
    {
      onSwitchtrackFlag = true;
      isSwitchtrackOccupied = true;
      switchtrackResetted = false;
      resetSwitchtrackFlag = false;

      targetSpeed = 500;
      manualRotateTarget = "station";

      client.publish("rollercoaster/event", "rotating_switchtrack_to_station");
      stabilizationStarted = false; 
    }
  }

  // 2. RELEASE TO STATION
  // We voegen '!isSwitchtrackMoving' en een check op de targetSpeed toe.
  // De trein mag PAS vertrekken als de track NIET meer beweegt en de sensor ECHT verbinding ziet.
  if (onSwitchtrackFlag && isNextBlockFree && hallSensorStationConnectState && !isSwitchtrackMoving && !releaseTriggered && !switchtrackSafetyFlag)
  {
    targetPos = 0; // Open servo
    releaseswitchMotorState = true;
    releaseTriggered = true; // Deze vlag blokkeert dat we deze IF opnieuw uitvoeren
    client.publish("rollercoaster/event", "released_train_switchtrack");
  }

  // 3. RESET SWITCHTRACK
  // Deze mag PAS triggeren als de trein daadwerkelijk de switchtrack heeft verlaten EN in het station is.
  // Ik heb '!hallSensorOnSwitchtrackState' toegevoegd als extra check.
  if (releaseTriggered && trainInStationFlag && !hallSensorOnSwitchtrackState && !resetSwitchtrackFlag && !switchtrackSafetyFlag)
  {
    client.publish("rollercoaster/event", "resetting_switchtrack");
    resetSwitchtrackFlag = true;

    targetPos = 90; // Sluit servo
    releaseswitchMotorState = false;
    targetSpeed = -500; // Terug naar brakes
    manualRotateTarget = "brakes";
  }

  // 4. END LOGIC (Terug bij de brakes)
  if (resetSwitchtrackFlag && hallSensorBrakeConnectState && !isSwitchtrackMoving && !switchtrackResetted)
  {
    switchtrackResetted = true;
    isSwitchtrackOccupied = false;
    onSwitchtrackFlag = false;
    releaseTriggered = false;
    resetSwitchtrackFlag = false; // Reset deze voor de volgende rit
    targetSpeed = 0;
    client.publish("rollercoaster/event", "switchtrack_resetted");
    client.publish("rollercoaster/block/event", "switchtrack_free");
  }

  // switchtrack free on start
  if (hallSensorBrakeConnect && !isSwitchtrackOccupied && !isSwitchtrackMoving && !switchtrackFreeFlag)
  {
    client.publish("rollercoaster/block/event", "switchtrack_free");
    switchtrackFreeFlag = true;
  }

  // SAFETY: Nooit draaien als de release-servo open staat!
  if (isSwitchtrackMoving && releaseswitchMotorState)
  {
    targetSpeed = 0; // Direct stoppen
    switchtrackSafetyFlag = true;
    client.publish("rollercoaster/event", "CRITICAL: Switchtrack tried to move while servo was OPEN!");
  }

  // SAFETY: Nooit releasen als we niet 100% verbonden zijn met het station!
  if (releaseswitchMotorState && !hallSensorStationConnectState)
  {
    targetPos = 90; // Servo direct dicht!
    switchtrackSafetyFlag = true;
    client.publish("rollercoaster/event", "CRITICAL: Emergency servo close - Station alignment lost!");
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

  pinMode(hallSensorStationConnect, INPUT_PULLUP);
  pinMode(hallSensorBrakeConnect, INPUT_PULLUP);
  pinMode(hallSensorMiddle, INPUT_PULLUP);
  pinMode(hallSensorOnSwitchtrack, INPUT_PULLUP);

  rotatingStepper.setMaxSpeed(700);
  rotatingStepper.setAcceleration(400);

  tiltingStepper.setMaxSpeed(400);
  tiltingStepper.setAcceleration(200);

  releaseServo.attach(SERVO_PIN);
}

void loop()
{
  if (!client.connected())
    connectMQTT();
  client.loop();

  updateSensors();
  publishHeartbeat();
  publishStatusIfChanged();

  handleMovement();

  if (coasterDispatched)
  {
    handleSwitchtrackBlockV2();
  }

  moveServoSmooth(releaseServo, currentPos, targetPos);
}
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

// === Bewegings-instellingen ===
const int TILT_FREE_STEPS = 150;      // Stappen om los te komen van de huidige guiding rail
const int TILT_SEAT_STEPS = 200;      // Stappen om in de nieuwe guiding rail te plaatsen
const float ROTATE_SPEED = 400.0;     // Rotatiesnelheid (steps/sec)
const float TILT_FREE_SPEED = 250.0;  // Tiltsnelheid tijdens vrijkomen
const float TILT_SEAT_SPEED = 150.0;  // Langzame snelheid voor zachte plaatsing in guiding rail

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
  PHASE_TILT_FREE,  // Tilt weg van huidige guiding rail
  PHASE_ROTATE,     // Draai naar doelstelling (sensor-gestuurd)
  PHASE_TILT_SEAT,  // Tilt zacht in nieuwe guiding rail
};

void handleMovement()
{
  static MovementPhase phase = PHASE_IDLE;
  static int tiltDir = 0;

  // Emergency stop: targetSpeed op 0 gezet van buitenaf tijdens beweging
  if (targetSpeed == 0 && phase != PHASE_IDLE)
  {
    rotatingStepper.stop();
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

  // --- START: nieuwe beweging ---
  if (phase == PHASE_IDLE)
  {
    tiltDir = (targetSpeed > 0) ? 1 : -1;
    isSwitchtrackMoving = true;

    tiltingStepper.setMaxSpeed(TILT_FREE_SPEED);
    tiltingStepper.setAcceleration(200);
    tiltingStepper.move(tiltDir * TILT_FREE_STEPS);
    phase = PHASE_TILT_FREE;
    return;
  }

  // --- FASE 1: Tilt vrij van huidige guiding rail ---
  if (phase == PHASE_TILT_FREE)
  {
    tiltingStepper.run();
    if (tiltingStepper.distanceToGo() == 0)
    {
      rotatingStepper.setMaxSpeed(ROTATE_SPEED);
      rotatingStepper.setSpeed(tiltDir * ROTATE_SPEED);
      phase = PHASE_ROTATE;
    }
    return;
  }

  // --- FASE 2: Roteer naar doelstelling ---
  if (phase == PHASE_ROTATE)
  {
    bool arrived = (tiltDir > 0) ? hallSensorStationConnectState : hallSensorBrakeConnectState;

    if (arrived)
    {
      rotatingStepper.stop();
      tiltingStepper.setMaxSpeed(TILT_SEAT_SPEED);
      tiltingStepper.setAcceleration(100);
      tiltingStepper.move(tiltDir * TILT_SEAT_STEPS);
      phase = PHASE_TILT_SEAT;
      return;
    }

    rotatingStepper.runSpeed();
    return;
  }

  // --- FASE 3: Tilt zacht in nieuwe guiding rail ---
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
        lastTiltStep = rotatingStepper.currentPosition(); // Reset lastTiltStep zodat hij niet meteen een enorme sprong maakt als de motor lang stil stond
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
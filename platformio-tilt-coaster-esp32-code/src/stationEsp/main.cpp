#include <WiFi.h>
#include <WebServer.h>
#include <AccelStepper.h>

// === WiFi Config ===
const char* ssid = "wieditleestisZOT";
const char* password = "jemama123";

// === Server ===
WebServer server(80);

// === Motor Config ===
#define STATION_IN1 18
#define STATION_IN2 19
#define STATION_IN3 22
#define STATION_IN4 23

#define LIFTHILL_IN1 13
#define LIFTHILL_IN2 12
#define LIFTHILL_IN3 14
#define LIFTHILL_IN4 27

AccelStepper stationStepper(AccelStepper::FULL4WIRE, STATION_IN1, STATION_IN3, STATION_IN2, STATION_IN4); 
AccelStepper liftStepper(AccelStepper::FULL4WIRE, LIFTHILL_IN1, LIFTHILL_IN3, LIFTHILL_IN2, LIFTHILL_IN4);


// === Sensors ===
#define hallSensorExitStation 35
#define hallSensorBottomLifthill 33
#define hallSensorEnterStation 34
#define hallSensorStartPosition 32

bool hallSensorExitStationState = false;
bool hallSensorBottomLifthillState = false;
bool hallSensorEnterStationState = false;
bool hallSensorStartPositionState = false;

bool hallSensorExitStationStateWeb = false;
bool hallSensorBottomLifthillStateWeb = false;
bool hallSensorEnterStationStateWeb = false;
bool hallSensorStartPositionStateWeb = false;

// === States ===
enum CoasterState {
  STATE_IDLE,
  STATE_DISPATCHING,
  STATE_TO_LIFTHILL,
  STATE_CLIMBING,
  STATE_RIDING,
  STATE_ENTER_STATION,
  STATE_ERROR
};

CoasterState currentState = STATE_IDLE;
String currentStateName = "IDLE";

void setState(CoasterState newState) {
  currentState = newState;
  switch (newState) {
    case STATE_IDLE: currentStateName = "IDLE"; break;
    case STATE_DISPATCHING: currentStateName = "DISPATCHING"; break;
    case STATE_TO_LIFTHILL: currentStateName = "TO_LIFTHILL"; break;
    case STATE_CLIMBING: currentStateName = "CLIMBING"; break;
    case STATE_RIDING: currentStateName = "RIDING"; break;
    case STATE_ENTER_STATION: currentStateName = "ENTER_STATION"; break;
    case STATE_ERROR: currentStateName = "ERROR"; break;
  }
  Serial.println("[STATE] → " + currentStateName);
}

// === State control vars ===
bool coasterDispatched = false;
bool manualMode = false;
bool stationMotorManual = false;
bool liftMotorManual = false;

// === Debug helper ===
// unsigned long lastDebugPrint = 0;
// void printMotorStatus() {
//   unsigned long now = millis();
//   if (now - lastDebugPrint < 500) return; // print max 2x per sec
//   lastDebugPrint = now;

//   Serial.print("[MOTOR] Station pos: "); Serial.print(stationStepper.currentPosition());
//   Serial.print(" / Target: "); Serial.print(stationStepper.targetPosition());
//   Serial.print(" | Lift pos: "); Serial.print(liftStepper.currentPosition());
//   Serial.print(" / Target: "); Serial.print(liftStepper.targetPosition());

//   Serial.print(" | Sensors: Exit "); Serial.print(hallSensorExitStationState);
//   Serial.print(", Bottom "); Serial.print(hallSensorBottomLifthillState);
//   Serial.print(", Enter "); Serial.print(hallSensorEnterStationState);
//   Serial.print(", StartPos "); Serial.println(hallSensorStartPositionState);

//   Serial.print(" | Mode: "); Serial.print(manualMode ? "MANUAL" : "AUTO");
//   Serial.print(" | Dispatched: "); Serial.println(coasterDispatched);
// }

unsigned long lastDebug = 0;

void updateSensors() {
  bool exitS = digitalRead(hallSensorExitStation) == LOW;
  bool bottomS = digitalRead(hallSensorBottomLifthill) == LOW;
  bool enterS = digitalRead(hallSensorEnterStation) == LOW;
  bool startS = digitalRead(hallSensorStartPosition) == LOW;

  if(exitS != hallSensorExitStationState || 
     bottomS != hallSensorBottomLifthillState || 
     enterS != hallSensorEnterStationState || 
     startS != hallSensorStartPositionState) {

    hallSensorExitStationState = exitS;
    hallSensorBottomLifthillState = bottomS;
    hallSensorEnterStationState = enterS;
    hallSensorStartPositionState = startS;
  }
}

// === Auto Dispatch ===
void handleDispatch() {
  if (manualMode) {
    server.send(400, "application/json", "{\"error\":\"manual mode active\"}");
    Serial.println("[DISPATCH] Manual mode active, dispatch blocked.");
    return;
  }
  coasterDispatched = true;
  setState(STATE_DISPATCHING);
  Serial.println("[DISPATCH] Dispatch initiated.");
  stationStepper.move(100000);
  server.send(200, "application/json", "{\"dispatch\":\"started\"}");
}

//Helper functie motor stop (dit moet op deze manier omdat gewoon een stop() doen met deze library de motor laat uitlopen en niet onmiddelijk stopt)
void StopStepperMotor(AccelStepper& motor) {
  motor.setCurrentPosition(motor.currentPosition());
  motor.moveTo(motor.currentPosition());
}

// === Auto Logic ===
void handleCoasterControl() {
  switch (currentState) {
    case STATE_IDLE:
      if (coasterDispatched) {
        setState(STATE_DISPATCHING);
        Serial.println("[AUTO] Station motor moving (DISPATCHING)");
      }
      break;

    case STATE_DISPATCHING:
        stationStepper.run();
        hallSensorStartPositionStateWeb = false;
    if (hallSensorExitStationState) {
        StopStepperMotor(stationStepper);
        setState(STATE_TO_LIFTHILL);
        hallSensorExitStationStateWeb = true;
        Serial.println("[AUTO] Station motor stopped, TO_LIFTHILL");
      }
      break;

    case STATE_TO_LIFTHILL:
      if (hallSensorBottomLifthillState) {
        setState(STATE_CLIMBING);
        
        // 1. ZET HET DOEL EENMALIG: Stel het aantal stappen voor de klim in.
        // Dit moet gebeuren net voordat de motor begint te klimmen.
        // De lift zal nu gecontroleerd bewegen met acceleratie/snelheid ingesteld in setup.
        liftStepper.move(-10000); 
        
        hallSensorBottomLifthillStateWeb = true;
        Serial.println("[AUTO] Lift motor starting CLIMBING (Goal 5000 steps set)");
      }
      break;

    // --- GECORRIGEERDE LOGICA VOOR NAUWKEURIGE STAPPER BESTURING ---
    case STATE_CLIMBING:
      // 2. ONDERHOUD DE BEWEGING: run() moet ELKE keer in de loop worden aangeroepen.
      // Het zorgt ervoor dat de motor beweegt totdat het doel is bereikt.
      liftStepper.run();

      // 3. CONTROLEER OF HET DOEL BEREIKT IS: distanceToGo() == 0 is de check.
      if (liftStepper.distanceToGo() == 0) {
        setState(STATE_RIDING);
        Serial.println("[AUTO] Lift motor reached top, now RIDING");
      }
      break;

    case STATE_RIDING:
      if (hallSensorEnterStationState) {
        setState(STATE_ENTER_STATION);
        hallSensorEnterStationStateWeb = true;
        Serial.println("[AUTO] Entering station...");
      }
      break;

    case STATE_ENTER_STATION:
      // Zorg ervoor dat de stationmotor blijft bewegen totdat de eindsensor getriggerd wordt.
      // We gebruiken een groot doel en stoppen handmatig met de sensor.
      if (stationStepper.distanceToGo() == 0) {
        // Zet een groot doel om zeker te zijn dat de motor blijft lopen.
        stationStepper.move(100000); 
      }
      stationStepper.run(); // Voer de beweging uit
      
      if (hallSensorStartPositionState) {
        StopStepperMotor(stationStepper);
        setState(STATE_IDLE);
        coasterDispatched = false;
        hallSensorStartPositionStateWeb = true;
        hallSensorBottomLifthillStateWeb = false;
        hallSensorExitStationStateWeb = false;
        hallSensorEnterStationStateWeb = false;
        Serial.println("[AUTO] Coaster returned to IDLE");
      }
      break;

    default:
      break;
  }


  //printMotorStatus();
}


// === /auto-control/status ===
void handleAutoControlStatus() {
  String json = "{";
  json += "\"hallSensorExitStation\":" + String(hallSensorExitStationStateWeb ? "true" : "false");
  json += ",\"hallSensorBottomLifthill\":" + String(hallSensorBottomLifthillStateWeb ? "true" : "false");
  json += ",\"hallSensorEnterStation\":" + String(hallSensorEnterStationStateWeb ? "true" : "false");
  json += ",\"hallSensorStartPosition\":" + String(hallSensorStartPositionStateWeb ? "true" : "false");
  json += ",\"coasterDispatched\":" + String(coasterDispatched ? "true" : "false");
  json += ",\"manualMode\":" + String(manualMode ? "true" : "false");
  json += ",\"stationMotorManual\":" + String(stationMotorManual ? "true" : "false");
  json += ",\"liftMotorManual\":" + String(liftMotorManual ? "true" : "false");
  json += ",\"currentState\":\"" + currentStateName + "\"";
  json += "}";
  server.send(200, "application/json", json);

  Serial.println("[STATUS] Auto-control status requested");
}

// === /state ===
void handleStateOnly() {
  String json = "{\"state\":\"" + currentStateName + "\",\"mode\":\"" + String(manualMode ? "manual" : "auto") + "\"}";
  server.send(200, "application/json", json);
  Serial.println("[STATE] State requested: " + currentStateName);
}

// === Setup ===
void setup() {
  Serial.begin(115200);

  pinMode(hallSensorExitStation, INPUT_PULLUP);
  pinMode(hallSensorBottomLifthill, INPUT_PULLUP);
  pinMode(hallSensorEnterStation, INPUT_PULLUP);
  pinMode(hallSensorStartPosition, INPUT_PULLUP);

  // Motors
  stationStepper.setMaxSpeed(800.0);
  stationStepper.setSpeed(-600);
  stationStepper.setAcceleration(300.0);

  liftStepper.setMaxSpeed(1000.0);
  liftStepper.setSpeed(-800);
  liftStepper.setAcceleration(400.0);

  // WiFi connect
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected! IP: " + WiFi.localIP().toString());

  // === Manual Control Endpoints ===
  server.on("/manual/on", []() {
    manualMode = true;
    setState(STATE_IDLE);
    server.send(200, "application/json", "{\"manual\":\"enabled\"}");
    Serial.println("[MANUAL] Manual mode enabled");
  });

  server.on("/manual/off", []() {
    manualMode = false;
    StopStepperMotor(stationStepper);
    StopStepperMotor(liftStepper);
    stationMotorManual = false;
    liftMotorManual = false;
    setState(STATE_IDLE);
    server.send(200, "application/json", "{\"manual\":\"disabled\"}");
    Serial.println("[MANUAL] Manual mode disabled");
  });

  server.on("/manual/stationmotor/on", []() {
    if (manualMode) {
      stationMotorManual = true;
      stationStepper.setSpeed(600);
    }
    server.send(200, "application/json", "{\"stationmotor\":\"on\"}");
    Serial.println("[MANUAL] Station motor ON");
  });

  server.on("/manual/stationmotor/off", []() {
    stationMotorManual = false;
    stationStepper.stop();
    server.send(200, "application/json", "{\"stationmotor\":\"off\"}");
    Serial.println("[MANUAL] Station motor OFF");
  });

  server.on("/manual/lifthillmotor/on", []() {
    if (manualMode) {
      liftMotorManual = true;
      liftStepper.setSpeed(800);
    }
    server.send(200, "application/json", "{\"lifthillmotor\":\"on\"}");
    Serial.println("[MANUAL] Lift motor ON");
  });

  server.on("/manual/lifthillmotor/off", []() {
    liftMotorManual = false;
    liftStepper.stop();
    server.send(200, "application/json", "{\"lifthillmotor\":\"off\"}");
    Serial.println("[MANUAL] Lift motor OFF");
  });

  // Routes
  server.on("/dispatch/go", handleDispatch);
  server.on("/auto-control/status", handleAutoControlStatus);
  server.on("/state", handleStateOnly);

  server.begin();
  setState(STATE_IDLE);
}

// === Loop ===
void loop() {
  server.handleClient();
  updateSensors();

  if(manualMode){
    if (stationMotorManual) {
    stationStepper.runSpeed();
  }
  if (liftMotorManual) {
    liftStepper.runSpeed();
  }
  } else {
    handleCoasterControl();
    stationStepper.run();
    liftStepper.run();
  }
}


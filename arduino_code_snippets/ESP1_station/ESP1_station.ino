#include <Stepper.h>
#include <ESP32Servo.h>

#define hallSensorEnterStation     26
#define hallSensorStartPosition    25
#define hallSensorExitStation      33
#define hallSensorBottomLifthill   32
#define hallSensorTopLifthill      34

#define MAGNET_DETECTED LOW
#define NO_MAGNET HIGH

#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23

#define IN5 13
#define IN6 12
#define IN7 14
#define IN8 27

const int stepsPerRevolution = 2048;

Stepper lifthillStepper(stepsPerRevolution, IN1, IN3, IN2, IN4);
Stepper stationStepper(stepsPerRevolution, IN5, IN7, IN6, IN8);

int lifthillsteps = 0;


void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(hallSensorEnterStation, INPUT);
  pinMode(hallSensorStartPosition, INPUT);
  pinMode(hallSensorExitStation, INPUT);
  pinMode(hallSensorBottomLifthill, INPUT);
  pinMode(hallSensorTopLifthill, INPUT);

  lifthillStepper.setSpeed(10);
  stationStepper.setSpeed(10);
}

void loop() {

  EnterStation();

  Serial.println("");
  Serial.println("coaster IN start position, READY to START");
  Serial.println("press O to open gates");
  Serial.println("");

  WaitForInput('O', "Opening gates.........");
  delay(500);
  Serial.println("/////////////////////////////////////////");
  delay(500);
  Serial.println("Gates are open, press O to close");
  Serial.println("");

  WaitForInput('O', "Closing gates.........");
  delay(500);
  Serial.println("------------------------------------");
  Serial.println("Coaster ready for dispatch, press E to start");
  Serial.println("");

  WaitForInput('E', "Coaster is starting");

  ExitStation();

  Serial.println("Coaster dispatched");
  delay(1000);
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


void EnterStation()
{
  while (digitalRead(hallSensorEnterStation) == HIGH) {
    Serial.println("coaster NOT in STATION");
  }

  while (digitalRead(hallSensorStartPosition) == HIGH) {
    for (int i = 0; i < stepsPerRevolution; i++) {
      stationStepper.step(1);
      if (digitalRead(hallSensorStartPosition) == LOW)
        break;
    }
  }
}

void ExitStation() {
  for (int i = 0; i < 1; i++) {
    stationStepper.step(3000);
  }

   Serial.println("Waiting for coaster at bottom of lifthill...");

  while (digitalRead(hallSensorBottomLifthill) == HIGH) {
    Serial.println("coaster NOT ON LIFTHILL");
  }

  Serial.println("Coaster detected at bottom, starting lift...");

  while (lifthillsteps < 19000) {
    lifthillStepper.step(-1); 
    lifthillsteps++;
  }

  Serial.println("Coaster reached top of lifthill.");
}


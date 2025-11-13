#include <AccelStepper.h>

// === Motor Config ===
#define ROTATING_IN1 19
#define ROTATING_IN2 21
#define ROTATING_IN3 22
#define ROTATING_IN4 23

#define TILTING_IN1 14
#define TILTING_IN2 13
#define TILTING_IN3 15
#define TILTING_IN4 18

AccelStepper rotatingStepper(AccelStepper::FULL4WIRE, ROTATING_IN1, ROTATING_IN3, ROTATING_IN2, ROTATING_IN4);
AccelStepper tiltingStepper(AccelStepper::FULL4WIRE, TILTING_IN1, TILTING_IN3, TILTING_IN2, TILTING_IN4);

String inputMotor = "";
String inputSteps = "";
bool waitingForMotor = true;
bool waitingForSteps = false;
bool executing = false;
long steps = 0;

void setup() {
  Serial.begin(115200);

  rotatingStepper.setMaxSpeed(700);
  rotatingStepper.setAcceleration(400);
  tiltingStepper.setMaxSpeed(700);
  tiltingStepper.setAcceleration(400);

  Serial.println("=== Stepper Test ===");
  Serial.println("Typ 'rotating' of 'tilting' en druk op Enter:");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (waitingForMotor) {
      if (input == "rotating" || input == "tilting") {
        inputMotor = input;
        waitingForMotor = false;
        waitingForSteps = true;
        Serial.println("Hoeveel stappen moet de motor zetten?");
      } else {
        Serial.println("Ongeldige keuze. Typ 'rotating' of 'tilting':");
      }
    } 
    else if (waitingForSteps) {
      inputSteps = input;
      steps = inputSteps.toInt();
      if (steps == 0 && inputSteps != "0") {
        Serial.println("Voer een geldig getal in:");
      } else {
        waitingForSteps = false;
        executing = true;
      }
    }
  }

  if (executing) {
    Serial.print("Beweeg ");
    Serial.print(inputMotor);
    Serial.print(" motor met ");
    Serial.print(steps);
    Serial.println(" stappen...");

    AccelStepper* targetStepper = nullptr;
    if (inputMotor == "rotating") targetStepper = &rotatingStepper;
    else if (inputMotor == "tilting") targetStepper = &tiltingStepper;

    if (targetStepper) {
      targetStepper->move(steps);
      while (targetStepper->distanceToGo() != 0) {
        targetStepper->run();
      }
    }

    Serial.println("Beweging voltooid!");
    Serial.println("----------------------");
    Serial.println("Typ 'rotating' of 'tilting' voor een nieuwe test:");

    waitingForMotor = true;
    executing = false;
  }
}

#include <Stepper.h>

#define IN1 19
#define IN2 18
#define IN3 22
#define IN4 23

#define IN5 15
#define IN6 4
#define IN7 16
#define IN8 17

const int stepsRotateBay = 550;
const int stepsRotateTrack = 550;

Stepper rotateBayStepper(stepsRotateBay, IN1, IN3, IN2, IN4);
Stepper rotateTrackStepper(stepsRotateTrack, IN5, IN7, IN6, IN8);

String input = "";

void setup() {
  Serial.begin(115200);
  rotateBayStepper.setSpeed(10);      // Set a reasonable speed
  rotateTrackStepper.setSpeed(5);
  
  Serial.println("ESP32 Stepper Controller Ready.");
  Serial.println("Use format: bay forward 300");
  Serial.println("             track backward 200");
}

void loop() {
  if (Serial.available()) {
    input = Serial.readStringUntil('\n');
    input.trim();
    processCommand(input);
  }
}

void processCommand(String cmd) {
  cmd.toLowerCase();

  String part1 = "", part2 = "", part3 = "";
  int firstSpace = cmd.indexOf(' ');
  int secondSpace = cmd.indexOf(' ', firstSpace + 1);

  if (firstSpace != -1) {
    part1 = cmd.substring(0, firstSpace);
    if (secondSpace != -1) {
      part2 = cmd.substring(firstSpace + 1, secondSpace);
      part3 = cmd.substring(secondSpace + 1);
    } else {
      part2 = cmd.substring(firstSpace + 1);
    }
  } else {
    Serial.println("Invalid command format.");
    return;
  }

  int steps = part3.toInt();
  if (steps == 0 && part3 != "0") {
    Serial.println("Invalid step count.");
    return;
  }

  if (part1 == "bay") {
    if (part2 == "forward") {
      rotateBayStepper.step(steps);
      Serial.println("Bay motor rotated forward " + String(steps) + " steps.");
    } else if (part2 == "backward") {
      rotateBayStepper.step(-steps);
      Serial.println("Bay motor rotated backward " + String(steps) + " steps.");
    } else {
      Serial.println("Invalid direction for bay. Use 'forward' or 'backward'.");
    }

  } else if (part1 == "track") {
    if (part2 == "forward") {
      rotateTrackStepper.step(steps);
      Serial.println("Track motor rotated forward " + String(steps) + " steps.");
    } else if (part2 == "backward") {
      rotateTrackStepper.step(-steps);
      Serial.println("Track motor rotated backward " + String(steps) + " steps.");
    } else {
      Serial.println("Invalid direction for track. Use 'forward' or 'backward'.");
    }

  } else {
    Serial.println("Unknown motor. Use 'bay' or 'track'.");
  }
}

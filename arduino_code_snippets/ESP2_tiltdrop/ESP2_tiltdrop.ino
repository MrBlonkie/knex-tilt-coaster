#include <Stepper.h>
#include <ESP32Servo.h>

#define IN1 18
#define IN2 19
#define IN3 22
#define IN4 23

#define SERVO_PIN 25

#define hallSensorOnTiltdrop     32
#define hallSensorTiltdropClosed 27
#define hallSensorTiltdropOpen   34
#define hallSensorOffTiltdrop    26


const int stepsTiltTrack = 550;
Stepper tiltTrackStepper(stepsTiltTrack, IN1, IN3, IN2, IN4);
Servo releaseServo;

void setup() {
  Serial.begin(115200);
  releaseServo.attach(SERVO_PIN);

  pinMode(hallSensorOnTiltdrop, INPUT);
  pinMode(hallSensorTiltdropClosed, INPUT);
  pinMode(hallSensorTiltdropOpen, INPUT);
  pinMode(hallSensorOffTiltdrop, INPUT);

  tiltTrackStepper.setSpeed(5);

  Serial.println("System ready. Waiting for signal from MASTER ESP.");
}

void loop() {
  
  if (!IsTiltdropClosed()) {
  Serial.println("Aborting: Tiltdrop couldn't close.");
  return;
}

  EnterTiltdrop();

if (!Tiltdrop()) {
  Serial.println("Aborting: Tiltdrop couldn't open.");
  return;
}


  delay(2000);
  
  DropCoaster();
}

bool IsTiltdropClosed() {
  
  if (digitalRead(hallSensorTiltdropClosed) == LOW) {
    Serial.println("Tiltdrop CLOSED, ready for lifthill sequence");
    return true;
  }

  Serial.println("Tiltdrop is NOT CLOSED! trying to close now");
  for (int i = 0; i < 2048; i++) {
    tiltTrackStepper.step(1);

    if (digitalRead(hallSensorTiltdropClosed) == LOW) {
      Serial.println("Tiltdrop detected, doing extra steps to make sure it's closed");
      tiltTrackStepper.step(5);
      Serial.println("Tiltdrop CLOSED, ready for lifthill sequence");
      return true;
    }
  }

  Serial.println("ERROR: Failed to close Tiltdrop within step limit");
  return false;
}


void EnterTiltdrop()
{
  while (digitalRead(hallSensorOnTiltdrop) == HIGH) {
    Serial.println("coaster NOT on TILTDROP");
  }

  delay(500);
  Serial.println("coaster ON TILTDROP");
  delay(500);
  Serial.println("lifthill sequence stop");
  delay(500);

}

bool Tiltdrop()
{
  tiltTrackStepper.setSpeed(5);
  delay(500);
  
  for (int i = 0; i < 2048; i++) {
    tiltTrackStepper.step(-1);

    if (digitalRead(hallSensorTiltdropOpen) == LOW) {
      Serial.println("Tiltdrop detected, doing extra steps to make sure it's open");
      tiltTrackStepper.step(-30);
      Serial.println("Tiltdrop OPEN, ready for drop");
      return true;
    }
  }

  Serial.println("ERROR: Failed to open Tiltdrop within step limit");
  return false;

}

void DropCoaster()
{
  for(int posDegrees = 0; posDegrees <= 90; posDegrees++) {
    releaseServo.write(posDegrees);
    delay(10);
  }

  delay(1000);
  Serial.println("Coaster released");
  delay(2000);

  for(int posDegrees = 90; posDegrees >= 0; posDegrees--) {
    releaseServo.write(posDegrees);
    delay(20);
  }

  Serial.println("Servo gesloten.");
  delay(100);

  Serial.println("Track resetten...");
  tiltTrackStepper.setSpeed(12);
  delay(500);
  tiltTrackStepper.step(stepsTiltTrack);
  delay(500);

}

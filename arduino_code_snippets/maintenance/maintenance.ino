#include <Stepper.h>
#include <ESP32Servo.h>

#define IN1 19
#define IN2 18
#define IN3 22
#define IN4 23

#define IN5 15
#define IN6 4
#define IN7 16
#define IN8 17

#define SERVO_PIN 25

const int stepsRotateBay = 550;
const int stepsRotateTrack = 550;

Stepper rotateBayStepper(stepsRotateBay, IN1, IN3, IN2, IN4);
Stepper rotateTrackStepper(stepsRotateTrack, IN5, IN7, IN6, IN8);

Servo lockServo;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  lockServo.attach(SERVO_PIN);

  rotateBayStepper.setSpeed(5);
  rotateTrackStepper.setSpeed(5);



}

void loop() {
  // put your main code here, to run repeatedly:
  rotateBayStepper.step(1000);
  delay(500);
  rotateBayStepper.step(-1000);

  Serial.println("rotated bay");
  delay(500);

  rotateTrackStepper.step(200);
  delay(500);
  rotateTrackStepper.step(-200);
  Serial.println("rotated track");
  delay(200);
}

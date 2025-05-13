//servo motor condensator nodig!! op pin 25 (goede PWM pin nodig)

#include <Stepper.h>
#include <ESP32Servo.h>

const int stepsTiltTrack = 540;

Stepper tiltTrackStepper(stepsTiltTrack, 19, 12, 18, 13);

Servo releaseServo;

void setup() {

  Serial.begin(115200);

  releaseServo.attach(25); // pin waar servo op zit

}

void loop() {
  
  WaitForInput('T', "Tilting track");
  tiltTrackStepper.setSpeed(5);
  delay(500);
  tiltTrackStepper.step(-stepsTiltTrack);
  delay(500);
  Serial.println("------------------------------------");
  Serial.println("track is tilted, servo ready to release");
  Serial.println("------------------------------------");

  for(int posDegrees = 0; posDegrees <= 90; posDegrees++) {
    releaseServo.write(posDegrees);
    Serial.println(posDegrees);
  }

  delay(1000);
  Serial.println("coaster released");

  for(int posDegrees = 90; posDegrees >= 0; posDegrees--) {
    releaseServo.write(posDegrees);
    Serial.println(posDegrees);
    delay(20);
  }

  Serial.println("servo closed.....");
  delay(100);
  Serial.println("--------------------");
  Serial.println("track resetting");
  tiltTrackStepper.setSpeed(12);
  delay(500);
  tiltTrackStepper.step(stepsTiltTrack);
  delay(500);
  Serial.println("tilt track in start position");

}

void WaitForInput(char expectedChar, String succesMessage) {
   
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

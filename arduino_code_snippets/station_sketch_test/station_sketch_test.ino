#include <Stepper.h>

#define hallSensorEnterStation 21
#define hallSensorStartPosition 22
#define hallSensorExitStation 23


bool isCoasterDispatched = false;


const int stepsStation = 2048;

Stepper stationStepper(stepsStation, 19, 12, 18, 13);


void setup() {

  Serial.begin(115200);

  pinMode(hallSensorEnterStation, INPUT);
  pinMode(hallSensorStartPosition, INPUT);
  pinMode(hallSensorExitStation, INPUT);

  stationStepper.setSpeed(20);

}

void loop() {

  while(digitalRead(hallSensorStartPosition) == HIGH) {
    Serial.println("coaster NOT in START POSITION");
    delay(500);
  }

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
  isCoasterDispatched = true;

  for(int i = 0; i<10 ; i++) {
  liftHillStepper.step(stepsLiftHill);
  }
  
  delay(1000);

  Serial.println("");
  

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

void EnterStation() {

  while(digitalRead(hallSensorEnterStation) == HIGH) {
    Serial.println("coaster NOT in STATION");
    delay(500);
  }

  for(int i = 0; i< ; i++) {
  stationStepper.step(stepsStation);
  }

}

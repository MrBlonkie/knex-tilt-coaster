#define hallSensorEnterStation 21
#define hallSensorStartPosition 22
#define hallSensorExitStation 23

void setup() {

  Serial.begin(115200);

  pinMode(hallSensorEnterStation, INPUT);
  pinMode(hallSensorStartPosition, INPUT);
  pinMode(hallSensorExitStation, INPUT);

}

void loop() {

  while(digitalRead(hallSensorStartPosition) == HIGH) {
    Serial.println("coaster NOT in START POSITION");
    delay(500);
  }

  Serial.println("");
  Serial.println("coaster IN start position, READY to START");
  Serial.println("press O to open gates");

  WaitForInput('O', "Opening gates.........");
  delay(500);
  Serial.println("/////////////////////////////////////////");
  delay(500);
  Serial.println("Gates are open, press O to close");
  
  WaitForInput('0', "Closing gates.........");
  delay(500);
  Serial.println("------------------------------------");
  Serial.println("Coaster ready for dispatch, press E to start");
 
  WaitForInput('E', "Coaster is starting");

}


void WaitForInput(char expectedChar, String succesMessage) {
   
    expectedChar = toupper(expectedChar);e

    while (true) {
        if (Serial.available() > 0) {
            char input = Serial.read();
            input = toupper(input);

            if (input == expectedChar) { 
                Serial.println(succesMessage);
                break;
            } else {
              Serial.println("input doesnt match instructions, try again.");
            }
            
        }
    }
}

void DispatchCoaster() {
  
}

#include <Stepper.h>

#define hallSensorEnterStation 23
#define hallSensorStartPosition 22
#define hallSensorExitStation 21

bool isCoasterDispatched = false;

const int stepsStation = 2048;

Stepper stationStepper(stepsStation, 19, 12, 18, 13);

void setup()
{

  Serial.begin(115200);

  pinMode(hallSensorEnterStation, INPUT);
  pinMode(hallSensorStartPosition, INPUT);
  pinMode(hallSensorExitStation, INPUT);

  stationStepper.setSpeed(20);
}

void loop()
{

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
  isCoasterDispatched = true;

  for (int i = 0; i < 2; i++)
  {
    stationStepper.step(stepsStation);
  }

  delay(1000);

  Serial.println("");
}

void WaitForInput(char expectedChar, String succesMessage)
{

  expectedChar = toupper(expectedChar);

  while (true)
  {
    if (Serial.available() > 0)
    {
      char input = Serial.read();
      input = toupper(input);

      if (input == expectedChar)
      {
        Serial.println(succesMessage);
        break;
      }
    }
  }
}

void EnterStation()
{

  while (digitalRead(hallSensorEnterStation) == HIGH)
  {
    Serial.println("coaster NOT in STATION");
  }

  while (digitalRead(hallSensorStartPosition) == HIGH)
  {
    for (int i = 0; i < stepsStation; i++)
    {
      stationStepper.step(1);
      if (digitalRead(hallSensorStartPosition) == LOW)
        break;
    }
  }
}

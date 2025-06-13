#define HALL_SENSOR_PIN 32  // The GPIO pin connected to the Hall sensor

void setup() {
  Serial.begin(115200);  // Start serial communication
  pinMode(HALL_SENSOR_PIN, INPUT);  // Set Hall sensor pin as input
}

void loop() {
  int sensorValue = digitalRead(HALL_SENSOR_PIN);  // Read the sensor value

  if (sensorValue == HIGH) {
    Serial.println("Hall sensor is NOT activated");
  } else {
    Serial.println("Hall sensor is activated");
  }
  delay(500);  // Delay to make reading more readable
}


int sensorPin = A0;  // Analog input pin for the water sensor
int sensorValue = 0; // Variable to store sensor value

void setup() {
  Serial.begin(9600);  // Initialize serial communication
}

void loop() {
  sensorValue = analogRead(sensorPin);  // Read the analog sensor value
  Serial.print("Water Sensor Value: ");
  Serial.println(sensorValue);  // Print the sensor value to the serial monitor
  delay(1000);  // Wait for a second
}

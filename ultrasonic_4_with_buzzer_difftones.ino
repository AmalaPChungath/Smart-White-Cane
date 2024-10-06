#include <NewPing.h>

// Define the pins for the ultrasonic sensors
const int triggerPin1 = 19;
const int echoPin1 = 2;
const int triggerPin2 = 18;
const int echoPin2 = 3;
const int triggerPin3 = 17;
const int echoPin3 = 4;
const int triggerPin4 = 16;
const int echoPin4 = 5;

// Define the pin for the buzzer
const int buzzerPin = 6;

// Define the maximum distance we want to measure (in cm)
const int maxDistance = 200;

int sensorPin = A0;  // Analog input pin for the water sensor
int sensorValue = 0; // Variable to store sensor value

// Create NewPing objects for each sensor
NewPing sonar1(19, 2, maxDistance);
NewPing sonar2(18, 3, maxDistance);
NewPing sonar3(17, 4, maxDistance);
NewPing sonar4(16, 5, maxDistance);


void setup() {
  // Set up the pin for the buzzer
  pinMode(6, OUTPUT);

  Serial.begin(9600);
}

void loop() {
  // Read distances from the ultrasonic sensors
  unsigned int distance1 = sonar1.ping_cm();
  unsigned int distance2 = sonar2.ping_cm();
  unsigned int distance3 = sonar3.ping_cm();
  unsigned int distance4 = sonar4.ping_cm();

  Serial.print("Distance 1: ");
  Serial.print(distance1);
  Serial.print(" cm\t");
  Serial.print("Distance 2: ");
  Serial.print(distance2);
  Serial.print(" cm\t");
  Serial.print("Distance 3: ");
  Serial.print(distance3);
  Serial.print(" cm\t");
  Serial.print("Distance 4: ");
  Serial.println(distance4);

  sensorValue = analogRead(sensorPin);  // Read the analog sensor value
  Serial.print("Water Sensor Value: ");
  Serial.println(sensorValue);  // Print the sensor value to the serial monitor
  delay(1000);  // Wait for a second

  // Play different sounds based on the sensor that is triggered
  if (distance1 < 10) {
    playTone(6, 1000, 500,1); // 1000 Hz tone for 500 ms
  } else if (distance2 < 10) {
    playTone(6, 1500, 500,2); // 1500 Hz tone for 500 ms
  } else if (distance3 < 10) {
    playTone(6, 2000, 500,3); // 2000 Hz tone for 500 ms
  } else if (distance4 < 10) {
    playTone(6, 2500, 500,4); // 2500 Hz tone for 500 ms
  } else if (sensorValue > 50) {
    playTone(6, 2500, 500,5);

  delay(100); // Delay between readings
}
void playTone(int pin, int frequency, int duration, int divisor) {
  long delayAmount = frequency / (2 * divisor); // Adjust delay based on divisor

  for (long i = 0; i < duration; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(delayAmount);
    digitalWrite(pin, LOW);
    delayMicroseconds(delayAmount);
  }
}

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

// Define the pin for the buzzer and vibration motor
const int buzzerPin = 6;
const int motorPin = 7;  // Pin for the vibration motor

// Define the maximum distance we want to measure (in cm)
const int maxDistance = 200;

int sensorPin = A0;  // Analog input pin for the water sensor
int sensorValue = 0; // Variable to store sensor value

// Create NewPing objects for each sensor
NewPing sonar1(triggerPin1, echoPin1, maxDistance);
NewPing sonar2(triggerPin2, echoPin2, maxDistance);
NewPing sonar3(triggerPin3, echoPin3, maxDistance);
NewPing sonar4(triggerPin4, echoPin4, maxDistance);

void setup() {
  // Set up the pin for the buzzer and the motor
  pinMode(buzzerPin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  
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

  // Play different sounds and activate the motor based on the sensor that is triggered
  if (distance1 < 15 && distance1 !=0) {
    activateAlert(1000, 500); // 1000 Hz tone for 500 ms
  } else if (distance2 < 30 && distance2 !=0) {
    activateAlert(1500, 500); // 1500 Hz tone for 500 ms
  } else if (distance3 < 30 && distance3 !=0) {
    activateAlert(2000, 500); // 2000 Hz tone for 500 ms
  } else if (distance4 < 30 && distance4 !=0) {
    activateAlert(2500, 500); // 2500 Hz tone for 500 ms
  } else if (sensorValue > 50) {
    activateAlert(2500, 500); // Buzzer for high water level
  }

  delay(100); // Delay between readings
}

// Function to activate the buzzer and motor simultaneously
void activateAlert(int frequency, int duration) {
  // Activate both buzzer and motor
  digitalWrite(motorPin, HIGH);  // Turn on the motor
  playTone(buzzerPin, frequency, duration);  // Play tone on buzzer
  digitalWrite(motorPin, LOW);   // Turn off the motor after the tone
}

// Function to play a tone on the buzzer
void playTone(int pin, int frequency, int duration) {
  long delayAmount = 1000000L / frequency / 2; // Adjust delay for frequency
  long cycles = (long)frequency * duration / 1000; // Calculate cycles

  for (long i = 0; i < cycles; i++) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(delayAmount);
    digitalWrite(pin, LOW);
    delayMicroseconds(delayAmount);
  }
}

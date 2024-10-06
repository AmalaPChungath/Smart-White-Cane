#include <NewPing.h>

#define TRIG_PIN1 19
#define ECHO_PIN1 2
#define TRIG_PIN2 18
#define ECHO_PIN2 3
#define TRIG_PIN3 17
#define ECHO_PIN3 4
#define TRIG_PIN4 16
#define ECHO_PIN4 5

#define BUZZER_PIN 6  // Define the pin for the buzzer
#define MAX_DISTANCE 200  // Maximum distance to measure (in cm)
#define THRESHOLD_DISTANCE 10  // Distance threshold for triggering the buzzer (in cm)

NewPing sonar1(TRIG_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIG_PIN2, ECHO_PIN2, MAX_DISTANCE);
NewPing sonar3(TRIG_PIN3, ECHO_PIN3, MAX_DISTANCE);
NewPing sonar4(TRIG_PIN4, ECHO_PIN4, MAX_DISTANCE);

void setup() {
  Serial.begin(9600);  // Start the serial communication
  pinMode(BUZZER_PIN, OUTPUT);  // Set the buzzer pin as an output
  digitalWrite(BUZZER_PIN, LOW);  // Make sure the buzzer is off initially
}

void loop() {
  delay(50);  // Wait before taking the next reading
  
  // Get the distance for each sensor
  int distance1 = sonar1.ping_cm();
  int distance2 = sonar2.ping_cm();
  int distance3 = sonar3.ping_cm();
  int distance4 = sonar4.ping_cm();

  // Print the distances to the Serial Monitor
  Serial.print("Distance1: ");
  Serial.print(distance1);
  Serial.print(" cm\t");
  
  Serial.print("Distance2: ");
  Serial.print(distance2);
  Serial.print(" cm\t");

  Serial.print("Distance3: ");
  Serial.print(distance3);
  Serial.print(" cm\t");

  Serial.print("Distance4: ");
  Serial.print(distance4);
  Serial.println(" cm");

  // Check if any sensor detects an object within the threshold distance
  if (distance1 < THRESHOLD_DISTANCE || distance2 < THRESHOLD_DISTANCE || 
      distance3 < THRESHOLD_DISTANCE || distance4 < THRESHOLD_DISTANCE) {
    digitalWrite(BUZZER_PIN, HIGH);  // Turn on the buzzer
  } else {
    digitalWrite(BUZZER_PIN, LOW);   // Turn off the buzzer
  }

  delay(500);  // Wait before taking the next round of readings
}

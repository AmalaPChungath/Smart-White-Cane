#include <SoftwareSerial.h>

// Define the RX and TX pins for SoftwareSerial
SoftwareSerial gsm(9, 8); // RX, TX (adjust as needed for your setup)

void setup() {
  // Start the Serial Monitor
  Serial.begin(9600);
  // Start communication with the GSM module
  gsm.begin(9600);
  
  Serial.println("Initializing GSM module...");
  delay(1000);

  // Test AT Command
  sendATCommand("AT");
  delay(1000);

  // Set SMS Text Mode
  sendATCommand("AT+CMGF=1"); // Set SMS mode to text
  delay(1000);

  // Check Network Registration
  sendATCommand("AT+CREG?");
  delay(1000);

  // Check Signal Quality
  sendATCommand("AT+CSQ");
  delay(1000);

  // Check SIM Card Status
  sendATCommand("AT+CPIN?");
  delay(1000);

  // Send a Test SMS
  sendSMS("+918138801839", "Hello! This is a test message from Arduino.");
}

void loop() {
  // No need for anything here in this example
}

// Function to send an AT command and print the response
void sendATCommand(const char *command) {
  gsm.println(command);
  delay(500); // Short delay to wait for the GSM module to respond

  // Print the response to the Serial Monitor
  while (gsm.available()) {
    Serial.write(gsm.read());
  }
}

// Function to send an SMS message
void sendSMS(const char *phoneNumber, const char *message) {
  gsm.println("AT+CMGF=1"); // Set SMS text mode
  delay(1000);

  gsm.print("AT+CMGS=\""); // Send SMS command
  gsm.print(phoneNumber);
  gsm.println("\"");
  delay(1000);

  gsm.print(message); // Message content
  delay(1000);

  gsm.write(26); // ASCII code for CTRL+Z to send the SMS
  delay(1000);

  Serial.println("Message sent!");
}

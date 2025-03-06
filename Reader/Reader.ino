#include <SoftwareSerial.h>

SoftwareSerial hc12(10, 11); // HC-12 TX to D10, RX to D11

void setup() {
  Serial.begin(9600);   // Serial Monitor baud rate
  hc12.begin(9600);     // HC-12 baud rate
  
  Serial.println("Entering AT mode...");
  delay(500); // Wait for HC-12 to stabilize

  // Send AT command to check connection
  hc12.println("AT");
  delay(500);
  
  // Set frequency to 434.2 MHz
  hc12.println("AT+C042"); // HC-12 frequency command (434.2 MHz is C042)
  delay(500);
  
  // Check current frequency
  hc12.println("AT+RX");
  delay(500);
}

void loop() {
  while (hc12.available()) {
    Serial.write(hc12.read()); // Print HC-12 response
  }
  while (Serial.available()) {
    hc12.write(Serial.read()); // Send user input to HC-12
  }
}

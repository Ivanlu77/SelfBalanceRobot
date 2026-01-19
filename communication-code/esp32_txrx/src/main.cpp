#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial espSerial(2);

void setup() {
  Serial.begin(115200);
  espSerial.begin(115200, SERIAL_8N1, 19, 17); // RX=GPIO19, TX=GPIO17
}

void loop() {
  espSerial.println("Hello from ESP32!");
  Serial.println("Sent: Hello from ESP32!");
  delay(1000);
  if (espSerial.available()) {
    String receivedData = espSerial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(receivedData);
  }
}
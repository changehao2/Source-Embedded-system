#include <SPI.h>
#include <LoRa.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(5, 6); // RX, TX
String dataPackage;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Receiver");
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
 delay(1000);
 LoRa.setSpreadingFactor(8);
 LoRa.setSignalBandwidth(62.5E3);
 LoRa.crc();
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.print("Received packet '");
    while (LoRa.available()) { 
      dataPackage = (char)LoRa.read();
      Serial.print(dataPackage);
      mySerial.print(dataPackage);
    }
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    delay(50);
  } 
}

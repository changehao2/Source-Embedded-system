#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define sData 50
LiquidCrystal_I2C lcd(0x3F, 20, 4); // 0x27 for 1602 :)))
String dataPackage;

String h, t, c, dust;
char temp[sData];
void setup() {
  Serial.begin(9600);
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

  lcd.begin();
  lcd.backlight();
  lcd.setCursor(2, 1);
  lcd.print("WAIT FOR DATA....");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  int i = 0, k = 2;
  if (packetSize) {
    Serial.print("Received packet '");
    while (LoRa.available()) {
      temp[i] = (char)LoRa.read();
      ++i;
    }
    for (int j = 0; j < sData; j++) {
      Serial.print(temp[j]);
      if (temp[j] == ' ') break;
    }
    //LCD display
    if (temp[0] == '1')
    {

      lcd.setCursor(0, 0);
      lcd.print("Humidity: "); //13
      lcd.setCursor(0, 1);
      lcd.print("Temperature: ");
      lcd.setCursor(0, 2);
      lcd.print("CO: ");
      lcd.setCursor(0, 3);
      lcd.print("Dust density: ");

      // clear data LCD
      for (int t = 0; t < 3; t++) {
        lcd.setCursor(13, t);
        lcd.print("      "); // 6 col
      }
      lcd.setCursor(13, 0);
      for (; k < sData; k++) {
        if (temp[k] == '\n') break;
        lcd.print(temp[k]);
      }
      lcd.setCursor(13, 1);
      for (k += 1; k < sData; k++) {
        if (temp[k] == '\n') break;
        lcd.print(temp[k]);
      }
      lcd.setCursor(13, 2);
      for (k += 1; k < sData; k++) {
        if (temp[k] == '\n') break;
        lcd.print(temp[k]);
      }
      lcd.setCursor(13, 3);
      for (k += 1; k < sData; k++) {
        if (temp[k] == '\n') break;
        lcd.print(temp[k]);
      }
    }
    else if (temp[0] == '0'){
      lcd.clear();
      lcd.setCursor(4, 1);
      lcd.print("Warning!");
      lcd.setCursor(2, 2);
      lcd.print("C0 is not safe...");
      delay(10000);
    }
    else {
      lcd.setCursor(0, 0);
      lcd.print("Humidity: "); //13
      lcd.setCursor(0, 1);
      lcd.print("Temperature: ");
      lcd.setCursor(0, 2);
      lcd.print("CO: ");
      lcd.setCursor(0, 3);
      lcd.print("Dust density: ");    
    }

  }
  // Clear data array_
  for (int j = 0; j < sData; j++) {
    temp[j] = ' ';
  }
}






























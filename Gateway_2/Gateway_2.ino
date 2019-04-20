#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>
#include <String.h>
SoftwareSerial mySerial(D2, D1); // RX, TX
char auth[] = "8706293aa979429c9f7fa1fc1df3189c";
char ssid[] = "Kim  Hoang";
char pass[] = "01676109293";
BlynkTimer timer; 
String h, t, c, dust, check = "1";
void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);
  while(!Serial){
  }
   timer.setInterval(1000L, sendUptime); 

  Blynk.begin(auth, ssid, pass);
}

void sendUptime()
{
  if (mySerial.available()) {
    check = mySerial.readStringUntil('\n');
    if(check == "1"){
    h = mySerial.readStringUntil('\n');
    t = mySerial.readStringUntil('\n');
    c = mySerial.readStringUntil('\n');
    dust = mySerial.readStringUntil('\n');
    }
    else {
      Serial.println("WARNING _________________________!!!");
      Blynk.notify("Warning! C0 is not safe...");
    }
  }
  if(dust.length()){
  Serial.print("Humidity: ");
  Serial.println(h);
  Serial.print("Tempurature: ");
  Serial.println(t);
  Serial.print("Co2: ");
  Serial.println(c);
  Serial.print("Dust density: ");
  Serial.println(dust);  
  Blynk.virtualWrite(V1, h.toFloat());
  Blynk.virtualWrite(V2, t.toFloat());
  Blynk.virtualWrite(V3, c.toFloat());
  Blynk.virtualWrite(V4, dust.toFloat());     
  }
}

void loop()
{
  Blynk.run(); // all the Blynk magic happens here
  timer.run(); // BlynkTimer is working...
}


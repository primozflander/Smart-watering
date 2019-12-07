/*
Project: SmartWateringUI
Created: Primoz Flander 14.3.2019

v11: added NRF24 support

v10: fixed hour and minute setting

v9: begining of DEVELOPER version, added menu options, debug mode

v8: added startscreen version info, screensaver disable option

v7: added rtc, adaptive watering

v6: optimized code, added screensaver

v5: added luminosity sensor

v4: added ultrasonic sensor and heartbeat function

v3: added dht22 sensor

v2: added moisture sensor

v1: demo

Arduino pin   /      I/O:
DI2           ->     DT (or RF IRQ)
DI3           ->     CLK
DI4           ->     SW
DI5           ->     DHT22
DO6           ->     FET (Pump)
DO7           ->     FET (Grow light)
DI8           ->     Button (Manual watering)
DO9           ->     RF CE
DO10          ->     RF CS
DI11          ->     RF MOSI
DO12          ->     RF MISO
DO13          ->     RF SCK
A0            ->     Aout (Moisture sensor)
A4            ->     SDA (OLED)(GY-302)(rtc)
A5            ->     SCL (OLED)(GY-302)(rtc)

*/

/*=======================================================================================
                                    Includes
========================================================================================*/

#include <Rotary.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
//#include <NewPing.h>
#include <Wire.h>
#include <BH1750.h>
#include <DS3231.h>
#include <SPI.h>


/*=======================================================================================
                                    Definitions
========================================================================================*/
#define MY_DEBUG
#define MY_RADIO_RF24
#define CHILD_ID_HUM 0
#include <MySensors.h>

#define I2C_ADDRESS 0x3C // 0X3C+SA0 - 0x3C or 0x3D
#define RST_PIN -1
#define rotSwitch 4
#define hSensor  0
#define DHTPIN  5
#define DHTTYPE DHT22
#define pump  6
#define growLight  7
#define manualWat  8
//#define ledAlive  9
//#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor
//#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor
//#define pumpLed  13
//#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters)
#define menuItemsNr 13

/*=======================================================================================
                                User Configurarations
========================================================================================*/
MyMessage msg(CHILD_ID_HUM, V_HUM);

//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
Rotary r = Rotary(2, 3);
DS3231  rtc(SDA, SCL);
Time  tm;
DHT_Unified dht(DHTPIN, DHTTYPE);
SSD1306AsciiAvrI2c oled;
BH1750 lightMeter;

bool screenSaver;
bool wateringFlag = false;
int lastHumLevel;
int addr = 0;
int menuVal=3;
int menuLvl=3;
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin
int sensorValue = 0;
int wateringTime = 0;
int pAlive = 1;
int cycleAlive = 1;
int watLevel;
int maxPwm = 255;
int minPwm = 0;
int dayNumber = 0;
int wateringDay = 0;
float t = 0;
float h = 0;
float nAlive = 0;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
unsigned long nLoop = 0;
unsigned long screenLoop = 0;
unsigned long debugLoop = 0;
uint16_t lux;
// String menuItems[menuItemsNr] = {"<-Nazaj", "Cas cikla", "Vlaznost", "Zalivanje", "Ohr.zaslona", "Ura v.luci", "Ura zaliv.", "Kal.ura", "Kal.min"}; // max 11 letters
// String unitItems[menuItemsNr] = {"<-Nazaj", "sekund", "%", "sekund", "minut", "h", "h", "h", "min"};
String menuItems[menuItemsNr] = {"<-Nazaj", "Cas cikla", "Vlaznost", "Zalivanje", "Ohr.zaslona", "Ura v.luci", "Ura iz.luci", "Ura zaliv.", "Kal.ura", "Kal.min", "Int.zal", "Kal.temp.", "Kal.vlaz."}; // max 11 letters
String unitItems[menuItemsNr] = {"<-Nazaj", "sekund", "%", "sekund", "minut", "h", "h", "h", "h", "min", "dan/dni", "C", "%"};
String statusItems[2] = {"=)", "=("};
int valueItems[menuItemsNr];

void presentation()  
{ 
  sendSketchInfo("SmartWatering", "1.0");
  present(CHILD_ID_HUM, S_HUM); 
}

/*=======================================================================================
                                   Setup function
========================================================================================*/

void setup() {
  
  Serial.begin(115200);
  Wire.begin();
  lightMeter.begin();
  dht.begin();
  rtc.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);

  pinMode(rotSwitch, INPUT_PULLUP);
  pinMode(manualWat, INPUT_PULLUP);
  pinMode(pump, OUTPUT);
  pinMode(growLight, OUTPUT);
  //pinMode(ledAlive, OUTPUT);
  //pinMode(pumpLed, OUTPUT);

  digitalWrite(pump,LOW);
  digitalWrite(growLight,HIGH);
  //digitalWrite(ledAlive,LOW);

  //read from EEPROM
  for (int i=0; i < menuItemsNr; i++){
    valueItems[i] = (int8_t) EEPROM.read(addr+i);
  } 


  //display
  #if RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS, RST_PIN);
  #else // RST_PIN >= 0
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
  #endif // RST_PIN >= 0
  oled.setFont(Adafruit5x7);

  //interrupts
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();

//  //the following lines can be uncommented to set the date and time
//  rtc.setDOW(MONDAY);     // Set Day-of-Week to SUNDAY
//  rtc.setTime(16, 41, 0);     // Set the time to 12:00:00 (24hr format)
//  rtc.setDate(6, 11, 2017);   // Set the date to January 1st, 2014

  versionInfo();
  getMoisture();
  getDHT();
  showMenu();
}

/*=======================================================================================
                                            Loop
========================================================================================*/

void loop() {

  watering();
  readSw();
  loopTiming();
  //heartBeat();
  manual();
}

/*=======================================================================================
                                         Functions
========================================================================================*/

ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result == DIR_NONE) {

  }
  else if (result == DIR_CW) {
    screenLoop = 0; //reset screensaver timer
    screenSaver = false;
        
    if (menuLvl == 0) {
      menuLvl = 1;
      showMenu();
    }
    else if (menuLvl == 1)  {
      menuVal--;
      if (menuVal < 0)  {
        menuVal = (menuItemsNr-1);
      }
      showMenu();
    }
    else if (menuLvl == 2) {
      decVal();
    }   
  }
  
  else if (result == DIR_CCW) {
    screenLoop = 0; //reset screensaver timer
    screenSaver = false;
        
    if (menuLvl == 0) {
      menuLvl = 1;
      showMenu();
    }
    else if (menuLvl == 1)  {
      menuVal++;
      if (menuVal > (menuItemsNr-1))  {
        menuVal = 0;
      }
      showMenu();
    }
     else if (menuLvl == 2) {
      incVal();
    }
  }
}

void incVal() {

  valueItems[menuVal] ++;
  if ((menuVal == 5) || (menuVal == 6) || (menuVal == 7) || (menuVal == 8)) {
    if (valueItems[menuVal] > 23){
    valueItems[menuVal] = 23;
    } 
  }
  else if (menuVal == 9)  {
    if (valueItems[menuVal] > 59) {
    valueItems[menuVal] = 59;
    } 
  }
  else if ((menuVal == 11) || (menuVal == 12))  {
    if (valueItems[menuVal] > 30) {
    valueItems[menuVal] = 30;
    } 
  }
  else if (valueItems[menuVal] > 127) {
    valueItems[menuVal] = 127;
  }
  EEPROM.update(addr + menuVal, valueItems[menuVal]);
  showMenu();
}


void decVal() {
  valueItems[menuVal] --;
  if  ((menuVal == 11) || (menuVal == 12))  {
    if (valueItems[menuVal] < -30)  {
      valueItems[menuVal] = -30;
    }
  }
  else if (valueItems[menuVal] < 0)  {
    valueItems[menuVal] = 0;
  }
  EEPROM.update(addr + menuVal, valueItems[menuVal]);
  showMenu();
}

void readSw() {

int reading = digitalRead(rotSwitch);

if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading; 
      if (buttonState == HIGH) {
        screenLoop = 0; //reset screensaver timer
        screenSaver = false;
        
        switch (menuLvl) {
          
          case 0:
            menuLvl = 1;
            showMenu();
          break;
          
          case 1:
            if (menuVal == 0) {
              menuLvl = 0;
              oled.clear();
            }
            else  {
              menuLvl = 2;
            }
            showMenu();
          break;
          
          case 2:
            if (menuVal == 8) {
              tm = rtc.getTime();
              rtc.setTime(valueItems[8], tm.min, 0);          
            }
            else if (menuVal == 9)  {
              tm = rtc.getTime();
              rtc.setTime(tm.hour, valueItems[9], 0);
            }
            menuLvl = 1;
            showMenu();
            
          break;
          
          default:
            menuLvl = 0;  // SET DEFAULT SCREEN
            showMenu();
          break;
        }
      }
    }
  }
  lastButtonState = reading;
}

void getMoisture()  {

  sensorValue = analogRead(hSensor);
  sensorValue = map(sensorValue,0,1023,100,0);
  send(msg.set(sensorValue));
}

void getDHT()  {

  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  t = event.temperature;
  dht.humidity().getEvent(&event);
  h = event.relative_humidity;  
}

void displayMenuItem()  {
  
  oled.set2X(); 
  oled.setCursor(0, 1);
  oled.println(menuItems[menuVal]);
  if (menuVal != 0) {
    oled.set1X(); 
    oled.println("----------------------");
    oled.println();
    oled.set2X(); 
    oled.print(valueItems[menuVal]);
    oled.set1X();
    oled.print(" ");
    oled.print(unitItems[menuVal]);
  }
}

void displayMainMenu()  {

  oled.set2X();
  oled.setCursor(0, 0);
  oled.print("Stanje: ");
  oled.println(plantStatus());
  oled.set1X();
  oled.println("---------------------");
  oled.print("Vlaznost:");
  oled.print(sensorValue);
  oled.print(" % / ");
  oled.print(valueItems[2]);
  oled.println(" %  ");
  oled.print("TH zraka:");
  oled.print(t + valueItems[11],0);
  oled.print(" C / ");
  oled.print(h + valueItems[12],0);
  oled.println(" % ");
//  oled.print("Nivo vode:");
//  watLevel = 15 - sonar.ping_cm();
//  if (watLevel > 0 && watLevel < 15)  {
//    oled.print(watLevel);
//    oled.println(" cm     ");
//  }
//  else  {
//    oled.println("NaN     ");
//  }
  oled.print("Osvetljenost:");
  lux = lightMeter.readLightLevel();
  oled.print(lux);
  oled.println(" lux   ");
  oled.print("Ura:");
  oled.print(tm.hour);
  oled.print(":");
  if (tm.min < 10)  {
    oled.print("0");
    oled.print(tm.min);  
  }
  else  {
    oled.print(tm.min);
  }
  oled.print(" f:");        // water flag monitoring
  oled.print(wateringFlag);
  oled.print(" d:");        // water day monitoring
  oled.print(wateringDay);
}

void showMenu() {

  if (menuLvl == 0 && !screenSaver)  {
    displayMainMenu();
  }

  else if (!screenSaver)  {
    oled.clear();
    displayMenuItem();
  }
}

void loopTiming() {

   nLoop++;
   screenLoop++;
   delay(1);
   if (nLoop > (1000 * valueItems[1])) {
      nLoop = 0;  //reset loop timer
      //menuLvl = 0;
      getDHT();
      getMoisture(); 
      tm = rtc.getTime();   // Get data from the DS3231   
      if (menuLvl == 0) {
        showMenu();
      }
   }
  
   // Grow light
   if ((tm.hour >= valueItems[5]) && (tm.hour <= valueItems[6])) {
    digitalWrite(growLight,true);
   }
   else {
    digitalWrite(growLight,false);
   }
   
   // Sreeensaver
   if (valueItems[4] != 0)  { //screensaver is enabled
     if (screenLoop > (60000 * valueItems[4]) && !screenSaver) {
      oled.clear();
      screenSaver = true;
     }
     else {
      screenLoop++;
     }
   }
}

//void heartBeat() {
//
//   nAlive += pAlive*1;
//   if (cycleAlive <= 1) {
//    analogWrite(ledAlive,nAlive);
//   }
//   else if (cycleAlive == 4)  {
//    cycleAlive = 0;
//   }
//   if (nAlive >= 255) {
//    pAlive = -1;
//   }
//   else if (nAlive <= 0)  {
//    pAlive = 1;
//    cycleAlive++;
//   }
//}

void watering() {

  if ((tm.hour == valueItems[7]) && wateringFlag) {
    if (sensorValue < valueItems[2]) {
      wateringTime = valueItems[3] * 1000 * (valueItems[2] - sensorValue);
      wateringFlag = false;
      dayNumber = tm.dow;
    }
  }
  else if (tm.dow != dayNumber)  {
    wateringDay++;
    dayNumber = tm.dow;
    if  (wateringDay >= valueItems[10]) {
      wateringFlag = true;
      wateringDay = 0;
    }
  }

  if (wateringTime > 0) {
    //digitalWrite(pumpLed,HIGH);
    digitalWrite(pump,HIGH);
    wateringTime--; 
  }
  else  {
    //digitalWrite(pumpLed,LOW);
    digitalWrite(pump,LOW);
  } 
}

void manual(void) {

  if (digitalRead(manualWat) == LOW)  {
    //digitalWrite(pumpLed,HIGH);
    digitalWrite(pump,HIGH);
    digitalWrite(growLight,HIGH);
    delay(5000);
    //digitalWrite(pumpLed,LOW);
    digitalWrite(pump,LOW);
    digitalWrite(growLight,LOW);
    wateringFlag = true;    
  }
}

String plantStatus(void) {
  
  if ((abs(sensorValue - valueItems[2]) < 5) && (lux > 150) && (watLevel > 1))  { 
    return statusItems[0];
  }
  else  {
    return statusItems[1];
  }
}

void versionInfo() {

  oled.set2X(); 
  oled.setCursor(0, 1);
  oled.println("Smart");
  oled.println("Watering");
  oled.set1X(); 
  oled.println("DEVELOPER");
  oled.print("ver11 2019");
  delay(1000);
}  

void serialDebug() {

//  Serial.print("debugLoop:");
//  Serial.println(debugLoop);
  debugLoop++;

  if  (debugLoop > 1000)  {
    Serial.print("screensaver:");
    Serial.println(screenSaver);
    Serial.println("wateringFlag:");
    Serial.print(wateringFlag);
    Serial.print("menuVal:");
    Serial.println(menuVal);
    Serial.print("menuLvl:");
    Serial.println(menuLvl);
    Serial.print("buttonState:");
    Serial.println(buttonState);
    Serial.print("lastButtonState:");
    Serial.println(lastButtonState);
    Serial.print("sensorValue:");
    Serial.println(sensorValue);
    Serial.print("wateringTime:");
    Serial.println(wateringTime);
    Serial.print("watLevel:");
    Serial.println(watLevel);
    Serial.print("dayNumber:");
    Serial.println(dayNumber);
    Serial.print("wateringDay:");
    Serial.println(wateringDay);
    Serial.print("t:");
    Serial.println(t);
    Serial.print("h:");
    Serial.println(h);
    Serial.print("nLoop:");
    Serial.println(nLoop);
    Serial.print("screenLoop:");
    Serial.println(screenLoop);
    Serial.print("lux:");
    Serial.println(lux);
    debugLoop = 0;
  } 
}

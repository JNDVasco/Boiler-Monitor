#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "time.h"
#include "EEPROM.h"
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>

//==== wifi Connectivity ====
const char *ssid = "YOUR SSID";
const char *password = "YOUR PASSWORD";

const char *ntpServer = "pool.ntp.org"; //Change with your own server or one you like
const long gmtOffset_sec = 0;
const int daylightOffset_sec = 3600;

const char *mqtt_server = "YOUR MQTT BROKER IP";
const int mqtt_port = 1883;
const char *mqtt_user = "YOUR MQTT USER";
const char *mqtt_password = "YOUR MQTT PASSWORD";

const String hostname = "THE HOST NAME YOU WANT";

WiFiClient espClient;
PubSubClient mqtt(espClient);

bool isConnecting = true;

//Topics
#define boilerStatus_topic "caldeira/caldeira"
#define burnerStatus_topic "caldeira/queimador"
#define pumpStatus_topic "caldeira/bomba"
#define boilerStartCount_topic "caldeira/caldeira/startCount"
#define burnerStartCount_topic "caldeira/queimador/startCount"
#define pumpStartCount_topic "caldeira/bomba/startCount"
#define boilerRuntime_topic "caldeira/caldeira/runtime"
#define burnerRuntime_topic "caldeira/queimador/runtime"
#define pumpRuntime_topic "caldeira/bomba/runtime"
#define temperature_topic "caldeira/temperatura"
#define fuelLeft_topic "caldeira/fuel/left"

#define payloadON "ON"
#define payloadOFF "OFF"

//==== Data Structures ====
const byte romAddr = 0x50; //A0 A1 A2 pulled to ground = 0x50

struct eepromAddresses //All writen vars are 4 bits size, so 16 space is more than enough
{
  int unixTime = 0x00;
  int boilerRunTime = 0x10;
  int pumpRunTime = 0x20;
  int burnerRunTime = 0x30;
  int fuelConsumption = 0x40;
  int fuelLeft = 0x50;
  int fuelLastReset = 0x60;
  int boilerStartCount = 0x70;
  int pumpStartCount = 0x80;
  int burnerStartCount = 0x90;
};

eepromAddresses addr;

//This struct is used just to convert unix time to a pleasing format
struct runTimeDurations
{
  time_t boiler = 0;
  time_t burner = 0;
  time_t pump = 0;
};
runTimeDurations runtime;

//Union to split the 4 byte variables into 1 byte chunks and store it on the EEPROM
union floatPack
{
  float data;
  uint8_t split[4];
};

union longPack
{
  unsigned long data;
  uint8_t split[4];
};

//This struct stores info about the fuel
struct fuelData
{
  floatPack capacity;
  floatPack left;
  floatPack consumption; //Fuel Comsumption in L/h
  longPack lastReset;
};

fuelData fuel;

//This is the main class of the program
class timeCounter
{
private:
  byte pin; //Pin where the relay is connected to

public:
  bool isOn;           //If the actuator is on or off
  longPack runtime;    //The time it was on
  longPack startCount; //The number of times it was started

  //Simple constructor, takes the pin and sets it as INPUT_PULLUP
  timeCounter(byte pin)
  {
    this->pin = pin;
    pinMode(this->pin, INPUT_PULLUP);
  };

  //Just a default destructor
  ~timeCounter() = default;

  //This function is quite crude but works, checks if the actuator is on
  //And increments the runtime if it is
  void updateRuntime() //Must be called only once a second!
  {
    if (this->checkIsOn())
    {
      this->runtime.data++;
    }
  }

  //Reads the input and tracks the changes from low to high
  bool checkIsOn()
  {
    if (!digitalRead(this->pin))
    {
      if (this->isOn == false) //If it is on now amd wasnt it means it was just turned on
      {
        startCount.data++;
        this->isOn = true;
      }
    }
    else
    {
      this->isOn = false;
    }

    return this->isOn;
  };
};

//==== Hardware ====
timeCounter boiler(27);
timeCounter burner(26);
timeCounter pump(25);

#define rightButton 19
#define upButton 18
#define downButton 17
#define leftButton 16

//==== DS18B20 Temp Sensor ====
#include <OneWire.h>
#include <DallasTemperature.h>

#define tempSensor 23
OneWire oneWire(tempSensor);
DallasTemperature sensor(&oneWire);
float sensorTempC = 0.0;

//==== LCD ====
LiquidCrystal_I2C lcd(0x27, 16, 2);

byte infoToShow = 0;

byte literHour[] = {
    B10000,
    B10000,
    B11011,
    B00100,
    B11000,
    B00101,
    B00111,
    B00101};

byte tempChar[] = {
    B11000,
    B11000,
    B00000,
    B00111,
    B01000,
    B01000,
    B01000,
    B00111};

//==== Time Vars ====
struct tm timeinfo;
longPack secondsUnix;

char timeBuffer[17];
char dateBuffer[17];

uint64_t currentMillis = 0;
uint64_t previousMillis = 0;

uint64_t previousMillisConnect = 8000;
uint64_t previousMillisUpdateEEPROM = 0;
uint64_t previousMillisReadInputs = 0;
uint64_t previousMillisChangeInfo = 0;
uint64_t previousMillisMenu = 0;

const int intervalChangeInfo = 8000;       //Change the info on the screen every 8 seconds
const long intervalEEPROMUpdate = 1200000; //update the EEPROM Every 20 minutes
const int intervalUpdateTime = 500;        //Update time every half a second

//==== Function declarations ====
void getTime();
void handleWifi(uint64_t millis);
void handleMQTT();
void updateLCD(struct runTimeDurations *input);
void sendValues();
void computeFuel();
void resetTimes();
void resetFuel();
void menu();
float getTemperature();

void updateEEPROM();
byte writeToEEPROM(byte device, unsigned int addr, byte *data, byte len);
void readEEPROM();
byte readFromEEPROM(byte device, unsigned int addr, byte *data, byte len);

//================================================================
//==== SETUP =====================================================
//================================================================
void setup()
{
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, literHour);
  lcd.createChar(1, tempChar);

  sensor.begin();

  pinMode(leftButton, INPUT_PULLUP);
  pinMode(upButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);
  pinMode(rightButton, INPUT_PULLUP);

  WiFi.setHostname(hostname.c_str());

  mqtt.setServer(mqtt_server, mqtt_port);

  fuel.capacity.data = 800.00;
  fuel.consumption.data = 24.5;

  readEEPROM();
  Serial.println("Started!!");
}

//================================================================
//==== LOOP ======================================================
//================================================================
void loop()
{
  currentMillis = millis();

  handleWifi(currentMillis);

  if (!digitalRead(leftButton) || !digitalRead(downButton) || !digitalRead(upButton) || !digitalRead(rightButton))
  {
    menu();
  }

  if (currentMillis - previousMillisUpdateEEPROM >= intervalEEPROMUpdate)
  {
    updateEEPROM();
    previousMillisUpdateEEPROM = currentMillis;
  }

  if (currentMillis - previousMillisReadInputs >= intervalUpdateTime)
  {
    getTime();
    previousMillisReadInputs = currentMillis;
  }

  if (currentMillis - previousMillisChangeInfo >= intervalChangeInfo)
  {
    infoToShow++;
    computeFuel();
    lcd.clear();
    if (infoToShow > 5)
    {
      infoToShow = 0;
    }
    previousMillisChangeInfo = currentMillis;
  }

  if (currentMillis - previousMillis >= 1000)
  {
    sendValues();

    boiler.updateRuntime();
    pump.updateRuntime();
    burner.updateRuntime();

    runtime.boiler = boiler.runtime.data;
    runtime.pump = pump.runtime.data;
    runtime.burner = burner.runtime.data;

    updateLCD(&runtime);
    previousMillis = currentMillis;
  }
}

//================================================================
//==== Other Functions ===========================================
//================================================================

//================================================================
//= This function handles the main lcd loop, prints all the info
//= on the lcd
//= Recieves nothing
//= Returns nothing
//= Status: Done
//================================================================
void getTime()
{
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }

  strftime(timeBuffer, 17, "%X", &timeinfo);       //%X Local representation works fine
  strftime(dateBuffer, 17, "%d/%m/%Y", &timeinfo); //%x looks bad, formated the way I like

  time_t now;

  time(&now);

  secondsUnix.data = now;

  // Serial.println(timeBuffer);
  // Serial.println(dateBuffer);
  // Serial.println(WiFi.RSSI());
  // Serial.println(secondsUnix);
  // Serial.println("\n");
}

//================================================================
//= This function connects to the wifi, and reastablishes a conn
//= when the connection is lost
//= Recieves the millis
//= Returns nothing
//= Status: Done
//================================================================
void handleWifi(unsigned long millis)
{
  if ((WiFi.status() != WL_CONNECTED) && isConnecting)
  {
    if (millis - previousMillisConnect > 10000)
    {
      WiFi.begin(ssid, password);
      Serial.printf("Connecting to %s\n", ssid);
      previousMillisConnect = currentMillis;
    }
  }
  else if (WiFi.status() == WL_CONNECTED && isConnecting)
  {
    Serial.println("Connected!!!\n");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    handleMQTT();
    isConnecting = false;
  }
  else if ((WiFi.status() != WL_CONNECTED) && !isConnecting)
  {
    isConnecting = true;
  }
}

//================================================================
//= This function connects to the MQTT Server
//= Recieves nothing
//= Returns nothing
//= Status:
//================================================================
void handleMQTT()
{
  while (!mqtt.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect

    if (mqtt.connect("Client", mqtt_user, mqtt_password))
    {
      Serial.println("Connected.");
      // client.subscribe(ledSetTopic);
    }
    else
    {
      Serial.print("failed, state=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//======================================================================
//= This function gets the temperature from the DS18B20
//= Recieves nothing
//= Returns a float with the temperature value in Celsius
//= Status: Done
//======================================================================
float getTemperature()
{
  sensor.requestTemperatures();
  return sensor.getTempCByIndex(0); //index 0 refers to the temperature sensor

  // Serial.print("Temperature: ");
  // Serial.println(returnVar); //print temp in celsius
}

//======================================================================
//= This function sends the data to the server over MQTT
//= Recieves nothing
//= Returns nothing
//= Status: Done
//======================================================================
void sendValues()
{
  mqtt.publish(pumpStatus_topic, String(pump.isOn ? payloadON : payloadOFF).c_str(), true);
  mqtt.publish(boilerStatus_topic, String(boiler.isOn ? payloadON : payloadOFF).c_str(), true);
  mqtt.publish(burnerStatus_topic, String(burner.isOn ? payloadON : payloadOFF).c_str(), true);

  mqtt.publish(pumpStartCount_topic, String(pump.startCount.data).c_str(), true);
  mqtt.publish(boilerStartCount_topic, String(boiler.startCount.data).c_str(), true);
  mqtt.publish(burnerStartCount_topic, String(burner.startCount.data).c_str(), true);

  mqtt.publish(pumpRuntime_topic, String(pump.runtime.data).c_str(), true);
  mqtt.publish(boilerRuntime_topic, String(boiler.runtime.data).c_str(), true);
  mqtt.publish(burnerRuntime_topic, String(burner.runtime.data).c_str(), true);

  mqtt.publish(fuelLeft_topic, String(fuel.left.data).c_str(), true);

  mqtt.publish(temperature_topic, String(getTemperature()).c_str(), true);
}

//======================================================================
//= This function prints the data on the LCD
//= Recieves the time the relays have been on
//= Returns nothing
//= Status: Done
//======================================================================
void updateLCD(struct runTimeDurations *input)
{
  static struct tm *auxTime;
  static char timeInfo[21];

  switch (infoToShow)
  {
  case 0:

    lcd.setCursor(0, 0);
    lcd.print("    ");
    lcd.print(timeBuffer);
    lcd.setCursor(0, 1);
    lcd.print("   ");
    lcd.print(dateBuffer);

    break;

  case 1:

    lcd.setCursor(0, 0);
    lcd.print("Cald: ");
    lcd.print(boiler.checkIsOn() ? "ON" : "OFF");
    lcd.print(" - ");
    lcd.print(boiler.startCount.data);
    lcd.print("   ");
    lcd.setCursor(0, 1);
    lcd.print("Tmp: ");
    auxTime = localtime(&input->boiler);
    snprintf_P(timeInfo, 21, PSTR("% 5dH%02dM%02d"), auxTime->tm_hour, auxTime->tm_min, auxTime->tm_sec);
    lcd.print(timeInfo);

    break;
  case 2:

    lcd.setCursor(0, 0);
    lcd.print("Queim: ");
    lcd.print(burner.checkIsOn() ? "ON" : "OFF");
    lcd.print(" - ");
    lcd.print(burner.startCount.data);
    lcd.print("   ");
    lcd.setCursor(0, 1);
    lcd.print("Tmp: ");
    auxTime = localtime(&input->burner);
    snprintf_P(timeInfo, 21, PSTR("% 5dH%02dM%02d"), auxTime->tm_hour, auxTime->tm_min, auxTime->tm_sec);
    lcd.print(timeInfo);

    break;
  case 3:

    lcd.setCursor(0, 0);
    lcd.print("Bomba: ");
    lcd.print(pump.checkIsOn() ? "ON" : "OFF");
    lcd.print(" - ");
    lcd.print(pump.startCount.data);
    lcd.print("   ");
    lcd.setCursor(0, 1);
    lcd.print("Tmp: ");
    auxTime = localtime(&input->pump);
    snprintf_P(timeInfo, 21, PSTR("% 5dH%02dM%02d"), auxTime->tm_hour, auxTime->tm_min, auxTime->tm_sec);
    lcd.print(timeInfo);

    break;
  case 4:

    lcd.setCursor(0, 0);
    lcd.print("Consumo: ");
    lcd.print(fuel.consumption.data);
    lcd.write(0);
    lcd.setCursor(0, 1);
    lcd.print("Disp: ");
    lcd.print(fuel.left.data);
    lcd.print("L");

    break;
  case 5:

    lcd.setCursor(0, 0);
    lcd.print("Temperatura: ");
    lcd.setCursor(5, 1);
    lcd.print(getTemperature());
    lcd.write(1);

    break;
  }
}

//======================================================================
//= This function handles the data update to the EEPROM
//= Recieves nothing
//= Returns nothing
//= Status: Done
//======================================================================
void updateEEPROM()
{
  // Serial.println("Long Data: ");
  // Serial.println(secondsUnix.data);
  // Serial.println("Split Data: ");
  // for (int i = 0; i < 4; i++)
  // {
  //   Serial.println(secondsUnix.split[i]);
  // }

  //Write time
  writeToEEPROM(romAddr, addr.unixTime, secondsUnix.split, sizeof(secondsUnix.split));

  //Write boiler on time
  writeToEEPROM(romAddr, addr.boilerRunTime, boiler.runtime.split, sizeof(boiler.runtime.split));

  //Write pump on time
  writeToEEPROM(romAddr, addr.pumpRunTime, pump.runtime.split, sizeof(pump.runtime.split));

  //Write burner on time
  writeToEEPROM(romAddr, addr.burnerRunTime, burner.runtime.split, sizeof(burner.runtime.split));

  //Write fuel consumption
  writeToEEPROM(romAddr, addr.fuelConsumption, fuel.consumption.split, sizeof(fuel.consumption.split));

  //Write fuel left
  writeToEEPROM(romAddr, addr.fuelLeft, fuel.left.split, sizeof(fuel.left.split));

  //Write the last fuel reset
  writeToEEPROM(romAddr, addr.fuelLastReset, fuel.lastReset.split, sizeof(fuel.lastReset.split));

  //Write the boiler start count
  writeToEEPROM(romAddr, addr.boilerStartCount, boiler.startCount.split, sizeof(boiler.startCount.split));

  //Write the pump start count
  writeToEEPROM(romAddr, addr.pumpStartCount, pump.startCount.split, sizeof(pump.startCount.split));

  //Write the burner start count
  writeToEEPROM(romAddr, addr.burnerStartCount, burner.startCount.split, sizeof(burner.startCount.split));

  Serial.println("\n[INFO] Updated EEPROM\n");
}

//======================================================================
//= This function handles the data read from the EEPROM
//= Recieves nothing
//= Returns nothing
//= Status: Done
//======================================================================
void readEEPROM()
{
  struct timeval tv;
  tv.tv_usec = 0;

  // unsigned long endLong = (buffer[0] << 8 | buffer[1]) << 16 | (buffer[2] << 8 | buffer[3]);

  //Read and set time
  readFromEEPROM(romAddr, addr.unixTime, secondsUnix.split, sizeof(secondsUnix.split));

  // Serial.println("Long read Data: ");
  // Serial.println(secondsUnix.data);
  // Serial.println("Split Data: ");
  // for (int i = 0; i < 4; i++)
  // {
  //   Serial.println(secondsUnix.split[i]);
  // }

  tv.tv_sec = secondsUnix.data;
  settimeofday(&tv, NULL);

  //Read the time the boiler was on
  readFromEEPROM(romAddr, addr.boilerRunTime, boiler.runtime.split, sizeof(boiler.runtime.split));

  //Read the time the burner was on
  readFromEEPROM(romAddr, addr.burnerRunTime, burner.runtime.split, sizeof(burner.runtime.split));

  //Read the time the pump was on
  readFromEEPROM(romAddr, addr.pumpRunTime, pump.runtime.split, sizeof(pump.runtime.split));

  //Read the fuel comsumption
  readFromEEPROM(romAddr, addr.fuelConsumption, fuel.consumption.split, sizeof(fuel.consumption.split));

  //Read the fuel left
  readFromEEPROM(romAddr, addr.fuelLeft, fuel.left.split, sizeof(fuel.left.split));

  //Read the last fuel reset
  readFromEEPROM(romAddr, addr.fuelLastReset, fuel.lastReset.split, sizeof(fuel.lastReset.split));

  //Read the boiler start count
  readFromEEPROM(romAddr, addr.boilerStartCount, boiler.startCount.split, sizeof(boiler.startCount.split));

  //Read the pump start count
  readFromEEPROM(romAddr, addr.pumpStartCount, pump.startCount.split, sizeof(pump.startCount.split));

  //Read the burner start count
  readFromEEPROM(romAddr, addr.burnerStartCount, burner.startCount.split, sizeof(burner.startCount.split));

  Serial.println("\n[INFO] Read EEPROM\n");
}

//======================================================================
//= This function reads the data on the EEPROM
//= Taken from Nick Gammon website -> http://gammon.com.au/i2c
//= Recieves the device address, the rom address, the data and lenght
//= Returns 0xFF means buffer too long
//=         other: other error (eg. device not present)
//= Status: Done
//======================================================================
byte writeToEEPROM(byte device, unsigned int addr, byte *data, byte len)
{
  byte err;
  byte counter;

  if (len > 32)  // 32 (in Wire.h)
    return 0xFF; // too long

  Wire.beginTransmission(device);
  Wire.write((byte)(addr >> 8));   // high order byte
  Wire.write((byte)(addr & 0xFF)); // low-order byte
  Wire.write(data, len);
  err = Wire.endTransmission();

  if (err != 0)
    return err; // cannot write to device

  // wait for write to finish by sending address again
  //  ... give up after 100 attempts (1/10 of a second)
  for (counter = 0; counter < 100; counter++)
  {
    delayMicroseconds(300); // give it a moment
    Wire.beginTransmission(device);
    Wire.write((byte)(addr >> 8));   // high order byte
    Wire.write((byte)(addr & 0xFF)); // low-order byte
    err = Wire.endTransmission();
    if (err == 0)
      break;
  }

  return err;

} // end of writeEEPROM

//======================================================================
//= This function reads the data on the EEPROM
//= Taken from Nick Gammon website -> http://gammon.com.au/i2c
//= Recieves the device address, the rom address, the data and lenght
//= Returns 0xFF means buffer too long
//=         0xFE means device did not return all requested bytes
//=         other: other error (eg. device not present)
//= Status: Done
//======================================================================
byte readFromEEPROM(byte device, unsigned int addr, byte *data, byte len)
{
  byte err;
  byte counter;

  if (len > 32)  // 32 (in Wire.h)
    return 0xFF; // too long

  Wire.beginTransmission(device);
  Wire.write((byte)(addr >> 8));   // high order byte
  Wire.write((byte)(addr & 0xFF)); // low-order byte
  err = Wire.endTransmission();

  if (err != 0)
    return err; // cannot read from device

  // initiate blocking read into internal buffer
  Wire.requestFrom(device, len);

  // pull data out of Wire buffer into our buffer
  for (counter = 0; counter < len; counter++)
  {
    data[counter] = Wire.read();
  }

  return 0; // OK
} // end of readEEPROM

//======================================================================
//= This function resets all counters and updates de EEPROM
//= Recieves nothing
//= Returns nothing
//= Status: Done
//======================================================================
void resetTimes()
{
  boiler.runtime.data = 0;
  pump.runtime.data = 0;
  burner.runtime.data = 0;

  boiler.startCount.data = 0;
  pump.startCount.data = 0;
  burner.startCount.data = 0;

  fuel.lastReset.data = 0;

  updateEEPROM();
}

//======================================================================
//= This function computes the remaining fuel
//= Recieves nothing
//= Returns nothing
//= Status: Done
//======================================================================
void computeFuel()
{
  /* To calculate the remaining fuel first calculate de burner runtime since the last reset
   * Then we need to convert it to hours from seconds, so we divide by 3600 (60 sec * 60 min)
   * After that we already have the runtime in hour, so we multiply by the consuption (in L\h)
   * To get the amount of fuel consumed since the last time.
   * Last, we subtract the consumed fuel from the tank capacity and we have the remaining fuel.
   * Since we can't have negative fuel we also set it to 0 if the result is less that 0 to mask any errors
   */

  fuel.left.data = (fuel.capacity.data - (fuel.consumption.data * ((burner.runtime.data - fuel.lastReset.data) / 3600.00)));

  if (fuel.left.data <= 0.0)
  {
    fuel.left.data = 0;
  }
}
//======================================================================
//= This function resets the fuel consumption. ie deposit refuel
//= Recieves nothing
//= Returns nothing
//= Status: Done
//======================================================================
void resetFuel()
{
  fuel.left.data = fuel.capacity.data;
  fuel.lastReset.data = burner.runtime.data;

  updateEEPROM();
}

//======================================================================
//= This function handles a small menu where we can reset things and set
//= The fuel consumption
//= Recieves nothing
//= Returns nothing
//= Status: Done
//======================================================================
void menu()
{
  byte option = 1;

  float fuelAux = fuel.consumption.data;

  delay(250);
  previousMillisMenu = millis();

  while (true)
  {
    currentMillis = millis();
    if (currentMillis - previousMillisMenu >= 8000)
    {
      return;
    }

    if (!digitalRead(rightButton))
    {
      option++;
      if (option > 4)
      {
        option = 1;
      }
      delay(250);
      previousMillisMenu = currentMillis;
    }

    if (!digitalRead(leftButton))
    {
      option--;
      if (option < 1)
      {
        option = 3;
      }
      delay(250);
      previousMillisMenu = currentMillis;
    }

    switch (option)
    {
    case 1:
    {
      lcd.setCursor(0, 0);
      lcd.print(" Reset runtimes ");
      lcd.setCursor(0, 1);
      lcd.print(" Yes: Press up  ");
      if (!digitalRead(upButton))
      {
        resetTimes();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("   Reset Done   ");
        delay(750);
        return;
      }
    }
    break;

    case 2:
    {
      lcd.setCursor(0, 0);
      lcd.print("   Reset Fuel   ");
      lcd.setCursor(0, 1);
      lcd.print(" Yes: Press up  ");
      if (!digitalRead(upButton))
      {
        resetFuel();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("   Reset Done   ");
        delay(750);
        return;
      }
    }
    break;

    case 3:
    {
      lcd.setCursor(0, 0);
      lcd.print("   Set Consumo  ");
      lcd.setCursor(0, 1);
      lcd.print(" Yes: Press up  ");
      if (!digitalRead(upButton))
      {
        lcd.setCursor(0, 1);
        lcd.print("                ");
        delay(250);
        while (true)
        {
          lcd.setCursor(5, 1);
          lcd.print(fuelAux);
          lcd.write(0);

          if (!digitalRead(upButton))
          {
            fuelAux += 0.1;
            delay(250);
          }

          if (!digitalRead(downButton))
          {
            fuelAux -= 0.1;
            if (fuelAux <= 0.0)
            {
              fuelAux = 0.0;
            }

            delay(250);
          }

          if (!digitalRead(rightButton))
          {
            fuel.consumption.data = fuelAux;
            updateEEPROM();
            lcd.clear();
            lcd.print("   Value Set!   ");
            delay(750);
            return;
          }

          if (!digitalRead(leftButton))
          {
            lcd.clear();
            lcd.print("   Not Set!   ");
            delay(750);
            return;
          }
        }
      }
    }
    break;

    case 4:
    {
      lcd.setCursor(0, 0);
      lcd.print("      Exit      ");
      lcd.setCursor(0, 1);
      lcd.print(" Yes: Press up  ");
      if (!digitalRead(upButton))
      {
        return;
      }
    }
    break;
    default:
      return;
      break;
    }
  }
}
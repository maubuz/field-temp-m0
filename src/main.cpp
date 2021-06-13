#include <Arduino.h>
// Debug variables
bool debugMode = false;

// ------------------------------------------
// Liquid Crystal I2C
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // library in PlatformIO
LiquidCrystal_I2C lcd(0x27, 20, 4); // library in Arduino IDE
int lcdTimeToClear = 2000;
// int lcdClearFlag = LOW;
int messageTime;

int dispRefreshRate = 500; // ms
int displayTimer = millis();

bool savedFlag = false;
int savedClearTime = 2000; // ms
int savedTimer;

// -------------------------------------------
// TSYS01 Temp Sensor
#include "Tsys01.h"

#define powerPin 19

#define chipSelect1 A1
#define chipSelect2 A2
#define chipSelect3 A3
#define chipSelect4 A4

Tsys01 sensor1;
Tsys01 sensor2;
Tsys01 sensor3;
Tsys01 sensor4;

float temp1 = 0.0;
float temp2 = 0.0;
float temp3 = 0.0;
float temp4 = 0.0;

long startSensor;
bool readingDone = true;

// ------------------------------------------
// Debounce
#define BUTTON_PIN 10          // the number of the pushbutton pin
const int DEBOUNCE_DELAY = 30; // the debounce time; increase if the output flickers

int lastSteadyState = LOW;          // the previous steady state from the input pin
int lastFlickerableState = LOW;     // the previous flickerable state from the input pin
int currentState;                   // the current reading from the input pin
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled

// -----------------------------------------
// SD Card
#include <SPI.h>
#include <SD.h>
// Set the pins used
#define cardSelect 4
int cardBypassFlag = LOW;
File logfile;

// -----------------------------------------
// GPS
#include <Adafruit_GPS.h>
#define GPSSerial Serial1
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
uint32_t timer = millis();

// =========================================
// Helper Functions

bool buttonPressed(int btn)
// function to check if the button is pressed and deal with debouncing
{
  currentState = digitalRead(btn);
  if (currentState != lastFlickerableState)
  {
    lastDebounceTime = millis();
    lastFlickerableState = currentState;
  }

  if (millis() - lastDebounceTime > DEBOUNCE_DELAY)
  {
    if (currentState == LOW && lastSteadyState == HIGH)
    {
      lastSteadyState = currentState;
      return true;
    }
    lastSteadyState = currentState;
  }
  return false;
}

void sPrint(const String &text)
{
  if (debugMode)
  {
    Serial.println(text);
  }
}

// display an error code
void error(uint8_t errno)
{
  // const int blinkDelay = 100;

  while (1)
  {
    switch (errno)
    {
    case 2:
      lcd.setCursor(0, 2);
      lcd.print("Insert card & reset");
      lcd.setCursor(0, 3);
      lcd.print("or press for temp.");

      sPrint("Error initialising sd card. Press button to continue without it");
      break;

    default: // TODO
      break;
    }
    if (buttonPressed(BUTTON_PIN))
    {
      // debug only. remove when done.
      lcd.clear();
      lcd.home();
      lcd.print("SD card bypass.");

      cardBypassFlag = HIGH;
      break;
    }
  }
}

void gpsTimeToSerial()
{
  Serial.print("\nTime: ");
  if (GPS.hour < 10)
  {
    Serial.print('0');
  }
  Serial.print(GPS.hour, DEC);
  Serial.print(':');
  if (GPS.minute < 10)
  {
    Serial.print('0');
  }
  Serial.print(GPS.minute, DEC);
  Serial.print(':');
  if (GPS.seconds < 10)
  {
    Serial.print('0');
  }
  Serial.print(GPS.seconds, DEC);
  Serial.print('.');
  if (GPS.milliseconds < 10)
  {
    Serial.print("00");
  }
  else if (GPS.milliseconds > 9 && GPS.milliseconds < 100)
  {
    Serial.print("0");
  }
  Serial.println(GPS.milliseconds);
}

void gpsTimeToSd()
{
  // logfile.print(temp1);

  if (GPS.hour < 10)
  {
    logfile.print('0');
  }
  logfile.print(GPS.hour, DEC);
  logfile.print(':');
  if (GPS.minute < 10)
  {
    logfile.print('0');
  }
  logfile.print(GPS.minute, DEC);
  logfile.print(':');
  if (GPS.seconds < 10)
  {
    logfile.print('0');
  }
  logfile.print(GPS.seconds, DEC);

  // TODO - Decide if milliseconds need to be included. Remove code bellow otherwise.
  // logfile.print('.');
  // if (GPS.milliseconds < 10)
  // {
  //   logfile.print("00");
  // }
  // else if (GPS.milliseconds > 9 && GPS.milliseconds < 100)
  // {
  //   logfile.print("0");
  // }
  // logfile.print(GPS.milliseconds);
}

void gpsDateToSd()
{
  logfile.print("20");
  logfile.print(GPS.year, DEC);
  logfile.print('/');
  logfile.print(GPS.month, DEC);
  logfile.print('/');
  logfile.print(GPS.day, DEC);
}

void gpsFixQualityToSd()
{
  logfile.print((int)GPS.fix);
  logfile.print(" , ");
  logfile.print((int)GPS.satellites);
}

void gpsPositionToSd()
{
  logfile.print(GPS.latitudeDegrees, 4);
  logfile.print(", ");
  logfile.print(GPS.longitudeDegrees, 4);
}

void showSaved()
{
  lcd.setCursor(0, 3);
  lcd.print(F("Saved!"));

  savedFlag = true;
  savedTimer = millis();
}

void gpsFixToScreen()
{
  lcd.setCursor(14, 3);
  if (GPS.fix)
  {
    lcd.print(" Fix ");
    lcd.print(GPS.satellites);
  }
  else
  {
    lcd.print("No fix");
  }
}

// =======================================================
void setup()
{

  // LCD
  //  lcd.begin(20, 4); // initialize the lcd in PlatformIO

  lcd.init(); // initialize the lcd in Arduino IDE
  lcd.backlight();

  lcd.home(); // go home
  lcd.print("TEMP. & GPS LOGGER");
  lcd.setCursor(0, 2); // go to the next line
  lcd.print("Design by Tony & Mau");

  pinMode(13, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  delay(50);
  pinMode(chipSelect1, OUTPUT); //  Tsys01 Sensor
  delay(50);
  pinMode(chipSelect2, OUTPUT); //  Tsys01 Sensor
  delay(50);
  pinMode(chipSelect3, OUTPUT); //  Tsys01 Sensor
  delay(50);
  pinMode(chipSelect4, OUTPUT); //  Tsys01 Sensor

  // Set all CS pins high to avoid any floating logic
  delay(50);
  digitalWrite(chipSelect1, HIGH);
  delay(50);
  digitalWrite(chipSelect2, HIGH);
  delay(50);
  digitalWrite(chipSelect3, HIGH);
  delay(50);
  digitalWrite(chipSelect4, HIGH);
  delay(50);

  sensor1 = Tsys01(TSYS01_SPI, powerPin, chipSelect1);
  delay(50);
  sensor2 = Tsys01(TSYS01_SPI, powerPin, chipSelect2);
  delay(50);
  sensor3 = Tsys01(TSYS01_SPI, powerPin, chipSelect3);
  delay(50);
  sensor4 = Tsys01(TSYS01_SPI, powerPin, chipSelect4);
  delay(50);

  Serial.begin(115200); // For serial debug mode
  sPrint("Initialising setup");

  if (!SD.begin(cardSelect))
  {
    lcd.clear();
    lcd.home();
    lcd.print("Card init. failed!");
    error(2);
  }

  if (cardBypassFlag == LOW)
  {
    char filename[15];
    strcpy(filename, "/ANALOG00.CSV");
    for (uint8_t i = 0; i < 100; i++)
    {
      filename[7] = '0' + i / 10;
      filename[8] = '0' + i % 10;
      // create if does not exist, do not open existing, write, sync after write
      if (!SD.exists(filename))
      {
        break;
      }
    }
    logfile = SD.open(filename, FILE_WRITE);
    if (!logfile)
    {
      lcd.clear();
      lcd.home();
      lcd.print("Couldnt create ");
      lcd.setCursor(0, 1);
      lcd.print(filename);
      error(3);
    }
    lcd.clear();
    lcd.home();
    lcd.print("Writing to ");
    lcd.setCursor(0, 1);
    lcd.print(filename);
    delay(2000);

    pinMode(13, OUTPUT);
    lcd.setCursor(0, 3);
    lcd.print("Ready!");

    logfile.println("Date (GMT), Time (GMT), T1, T2, T3, T4, Fix, Satellites, Latitude,Longitude");
    sPrint("End of setup reached");
  }

  // GPS

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);

  delay(4000);

  // Ask for firmware version
  // GPSSerial.println(PMTK_Q_RELEASE);

  lcd.clear();
}

// ==========================================================
void loop()
{
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c)
      Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived())
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return;                       // we can fail to parse a sentence in which case we should just wait for another
  }

  if (millis() - timer > 3000)
  {
    timer = millis(); // reset the timer
    gpsTimeToSerial();
  }

  if (readingDone)
  {
    sensor1.startAdc();
    delay(10);
    sensor2.startAdc();
    delay(10);
    sensor3.startAdc();
    delay(10);
    sensor4.startAdc();

    startSensor = millis();
    readingDone = false;
  }
  else if (millis() - startSensor > 10)
  {
    temp1 = sensor1.readTemperature();
    temp2 = sensor2.readTemperature();
    temp3 = sensor3.readTemperature();
    temp4 = sensor4.readTemperature();
    readingDone = true;
  }

  if (millis() - displayTimer > dispRefreshRate)
  {
    displayTimer = millis();
    lcd.home();
    lcd.print("T1: ");
    lcd.print(temp1);

    lcd.setCursor(11, 0);
    lcd.print("T2: ");
    lcd.print(temp2);

    lcd.setCursor(0, 2);
    lcd.print("T3: ");
    lcd.print(temp3);

    lcd.setCursor(11, 2);
    lcd.print("T4: ");
    lcd.print(temp4);

    gpsFixToScreen();
  }

  if (buttonPressed(BUTTON_PIN))
  {
    messageTime = millis();
    if (cardBypassFlag == LOW)
    {
      sPrint(F("Saving to Card"));

      lcd.setCursor(0, 3);
      lcd.print(F("Saving"));

      gpsDateToSd();
      logfile.print(",");
      gpsTimeToSd();
      logfile.print(",");

      digitalWrite(8, HIGH); // TODO - do I really need this? remove if possible

      logfile.print(temp1);
      logfile.print(",");
      logfile.print(temp2);
      logfile.print(",");
      logfile.print(temp3);
      logfile.print(",");
      logfile.print(temp4);
      logfile.print(",");

      digitalWrite(8, LOW); // same as above

      gpsFixQualityToSd();
      logfile.print(",");

      gpsPositionToSd();
      // logfile.print(",");

      logfile.println("");
      logfile.flush();

      showSaved();
    }
    else if (cardBypassFlag == HIGH)
    {
      lcd.setCursor(0, 3);
      lcd.print(F("Card bypass selected!"));
    }
  }

  if ((millis() - savedTimer > savedClearTime) && savedFlag)
  {
    lcd.setCursor(0, 3);
    lcd.print("      ");
    savedFlag = false;
  }
}
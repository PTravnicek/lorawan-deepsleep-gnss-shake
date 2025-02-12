/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <DFRobot_MAX17043.h>
#include <Wire.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include "DFRobot_GNSS.h"
#include "driver/rtc_io.h"
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Define pins
#ifdef __AVR__
  #define ALR_PIN       2
#else
  #define ALR_PIN       D2
#endif

#define LORA_POWER_PIN      D13   // Pin number to control the LDO
#define GPS_POWER_PIN       D10   // Pin number to control the LDO


// Sleep configuration
#define uS_TO_S_FACTOR 1000000ULL // Conversion factor for micro seconds to seconds
#define DEFAULT_SLEEP_TIME 1200
#define TIMEOUT_DURATION 30000   // Timeout duration in milliseconds (30 s)

RTC_DATA_ATTR int customSleepDuration = DEFAULT_SLEEP_TIME; // seconds
RTC_DATA_ATTR bool wasNightSleep = false; 
RTC_DATA_ATTR int  normalSleepDuration = DEFAULT_SLEEP_TIME;

// Battery and GNSS thresholds
#define BAT_PERCENTAGE        15

// GNSS Mode selection: Choose either INDOOR or OUTDOOR
enum Mode { INDOOR, OUTDOOR };
Mode currentMode = OUTDOOR;  // Set your desired mode here: INDOOR or OUTDOOR

// Mock coordinates for testing indoors
const double mockLatitude = 50.1234;   // Example latitude
const double mockLongitude = 14.1234;  // Example longitude
const double mockAltitude = 123.4;     // Example altitude in meters
int napTimeHour = 23;                  // Time when the unit goes to sleep for the whole night
int wakeUpHour = 6;                    // Time when the unit will wake up and follow normal work/sleep cycles

Adafruit_LIS3DH lis = Adafruit_LIS3DH();
// for 16G, try 5-10. We use 10 for less sensitivity.
#define CLICKTHRESHHOLD 10

#define LIS3DH_ADDRESS 0x18                // or 0x19 depending on your setup
#define WAKEUP_PIN GPIO_NUM_4  // Update this to match your wiring for INT1
#define SHAKE_TO_WAKE_ENABLED true  // Set to false to disable shake-to-wake functionality

// Definitions for INT1 configuration (using I2C)
#define LIS3DH_CTRL_REG3 0x22    // CTRL_REG3 controls which interrupts go to INT1
#define I1_CLICK_MASK    0x80    // Bit 7: I1_CLICK – enable click interrupt on INT1

#define LIS3DH_CTRL_REG5 0x24    // CTRL_REG5 contains latch control for INT1
#define LIR_INT1_MASK    0x08    // Bit 3: LIR_INT1 – latch interrupt request on INT1

// Enable high-pass filter for click detection. This filters out low-frequency motion.
#define LIS3DH_REG_CTRL2 0x21
void enableHPClick() {
  Wire.beginTransmission(LIS3DH_ADDRESS);
  Wire.write(LIS3DH_REG_CTRL2);
  Wire.endTransmission(false);
  Wire.requestFrom(LIS3DH_ADDRESS, 1);
  if (Wire.available()){
    uint8_t regVal = Wire.read();
    regVal |= 0x04; // Set HPCLICK bit (bit 2)
    Wire.beginTransmission(LIS3DH_ADDRESS);
    Wire.write(LIS3DH_REG_CTRL2);
    Wire.write(regVal);
    Wire.endTransmission();
  }
}

// Enable click interrupt on INT1 by writing CTRL_REG3
void enableINT1ClickInterrupt() {
  Wire.beginTransmission(LIS3DH_ADDRESS);
  Wire.write(LIS3DH_CTRL_REG3);
  Wire.write(I1_CLICK_MASK);
  Wire.endTransmission();
}

// Enable the latch on INT1 so the interrupt remains asserted until cleared
void enableINT1Latch() {
  Wire.beginTransmission(LIS3DH_ADDRESS);
  Wire.write(LIS3DH_CTRL_REG5);
  Wire.endTransmission(false);
  Wire.requestFrom(LIS3DH_ADDRESS, 1);
  if (Wire.available()) {
      uint8_t regVal = Wire.read();
      regVal |= LIR_INT1_MASK;
      Wire.beginTransmission(LIS3DH_ADDRESS);
      Wire.write(LIS3DH_CTRL_REG5);
      Wire.write(regVal);
      Wire.endTransmission();
  }
}

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
// In TTN the APPEUI is now called JOIN_EUI

// // // // // // // // // // // // // // // // // // // // 
// Připojení k IoT síti LoRa PILOT OP-24-00138-00001s01 //
// // // // // // // // // // // // // // // // // // // // 

// LoRaWAN keys and definitions
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x20, 0x00, 0x0D, 0x07, 0x00, 0x0E }; // 0E00070D00200000
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
// LSB FORMAT
static const u1_t PROGMEM DEVEUI[8]={ 0x6A, 0x38, 0x00, 0xD8, 0x7E, 0xD5, 0xB3, 0x70 }; // 70B3D57ED800386A
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// MSB FORMAT
static const u1_t PROGMEM APPKEY[16] = { 0xA6, 0x93, 0xA8, 0x1C, 0x8F, 0x3B, 0xB5, 0x28, 0x3D, 0x13, 0x65, 0xC3, 0xA6, 0xD8, 0xDA, 0x1F }; // A693A81C8F3BB5283D1365C3A6D8DA1F
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
static osjob_t sendjob;
const unsigned TX_INTERVAL = 60;

// LoRaWAN pin mapping
const lmic_pinmap lmic_pins = {
    .nss = D6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = D7,
    .dio = {D2, D3, D9}, // pins for DIO0, DIO1, DIO2
};

// Global variables
DFRobot_MAX17043 gauge;
uint8_t intFlag = 0;
DFRobot_GNSS_I2C gnss(&Wire, GNSS_DEVICE_ADDR);
bool gnssConnected = false;

RTC_DATA_ATTR int bootCount = 0;   // Variable to store boot count in RTC memory
int joinRetries = 0;
const int maxJoinRetries = 5; // Set the desired maximum number of retries

// Add RTC_DATA_ATTR variables to store last known GNSS data
RTC_DATA_ATTR double lastLatitude = 0.0;
RTC_DATA_ATTR double lastLongitude = 0.0;
RTC_DATA_ATTR double lastAltitude = 0.0;

// ============================ GNSS Utilities =============================
// Structure to hold GNSS data
struct GnssData {
    double latitude;
    double longitude;
    double altitude;
};

// Function to get GNSS data from the module
GnssData getGNSS() {
    GnssData data;
    sLonLat_t lat = gnss.getLat();
    sLonLat_t lon = gnss.getLon();
    data.latitude = lat.latitudeDegree;
    data.longitude = lon.lonitudeDegree;
    data.altitude = gnss.getAlt();
    return data;
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    default : Serial.println("Wakeup not caused by deep sleep"); break;
  }
}

// Function to set the interrupt flag high if conditions are fulfilled
void interruptCallBack()
{
  intFlag = 1;
}

// Function to read a register from the LoRa module
uint8_t readRegister(uint8_t addr) {
  digitalWrite(lmic_pins.nss, LOW);
  SPI.transfer(addr & 0x7F); // MSB=0 for read operation
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(lmic_pins.nss, HIGH);
  return val;
}

void enterSleepMode() {
  gnss.disablePower();
  WiFi.disconnect(true);
  btStop();

  digitalWrite(LORA_POWER_PIN, HIGH);    // turn the LORA module OFF (inverse logic to GNSS)
  // Convert pin 13 to RTC GPIO and hold it HIGH
  rtc_gpio_init((gpio_num_t)LORA_POWER_PIN);
  rtc_gpio_set_direction((gpio_num_t)LORA_POWER_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
  rtc_gpio_hold_dis((gpio_num_t)LORA_POWER_PIN);
  rtc_gpio_set_level((gpio_num_t)LORA_POWER_PIN, 1);
  rtc_gpio_hold_en((gpio_num_t)LORA_POWER_PIN);

  digitalWrite(GPS_POWER_PIN, LOW);    // turn the GNSS module OFF

  // Configure sleep timer
  esp_sleep_enable_timer_wakeup(customSleepDuration * uS_TO_S_FACTOR);

  esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, 1);  // 1 = wake on high level
  Serial.println("Setup ESP32 to sleep for " + String(customSleepDuration) + " Seconds");
  Serial.println("Going to sleep now");
  Serial.flush();  
  
  // Enter deep sleep
  esp_deep_sleep_start();
}

void printHex2(unsigned v) {
  v &= 0xff;
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);
}

// LoRaWAN event handling
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      joinRetries = 0; // Reset the retry counter
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i=0; i<sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i=0; i<sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      // Go to sleep failed join
      enterSleepMode();
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
          Serial.println(F("Received ack"));

      // If there's downlink data, parse it here
      if (LMIC.dataLen) {
          Serial.print(F("Received "));
          Serial.print(LMIC.dataLen);
          Serial.println(F(" bytes of payload"));

          // Example: parse the downlink as:
          // Byte[0] = command (e.g., 0x01 = change sleep time)
          // Byte[1..2] = new sleep time in minutes (uint16, big-endian)
          //
          // Adjust this to your own downlink format!

          uint8_t cmd     = LMIC.frame[LMIC.dataBeg + 0];
          uint8_t byteHi  = LMIC.frame[LMIC.dataBeg + 1];
          uint8_t byteLo  = LMIC.frame[LMIC.dataBeg + 2];

          // Convert 2 bytes into an integer, e.g., number of minutes
          uint16_t newSleepMinutes = (byteHi << 8) | byteLo;

          // If the command indicates "change sleep time"
          if (cmd == 0x01) {
              // Convert minutes to seconds, store in RTC variable
              customSleepDuration   = newSleepMinutes * 60;
              normalSleepDuration   = customSleepDuration;  // <--- store in RTC
              Serial.print("Downlink changed normal sleep to ");
              Serial.println(customSleepDuration);
          }
      }
      // Go to sleep after transmission
      enterSleepMode();
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      if (joinRetries < maxJoinRetries) {
        joinRetries++;
        Serial.print("Retrying join attempt ");
        Serial.print(joinRetries);
        Serial.print(" of ");
        Serial.println(maxJoinRetries);
        LMIC_startJoining();
      } else {
        Serial.println(F("Maximum join retries reached, going to sleep."));
        enterSleepMode();
      }
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

// Function to send data via LoRaWAN
void do_send(osjob_t* j){
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Read battery percentage from gauge
    int batteryPercentage = gauge.readPercentage();
    Serial.print("Battery percentage: ");
    Serial.print(batteryPercentage);
    Serial.println("%");

    if(intFlag == 1) {
      intFlag = 0;
      gauge.clearInterrupt();
      Serial.println("Low power alert interrupt!");
      // Handle low battery alert here
    }

    // Prepare data to send with payload size 10 bytes:
    // Byte 0: Battery percentage
    // Byte 1: Wake-up reason (esp_sleep_get_wakeup_cause())
    // Bytes 2-5: Latitude (32-bit integer)
    // Bytes 6-9: Longitude (32-bit integer)
    uint8_t mydata[10];
    mydata[0] = batteryPercentage;
    mydata[1] = (uint8_t) esp_sleep_get_wakeup_cause();

    // Add GNSS data:
    // If a fresh fix is available, use it and update saved values;
    // otherwise, use saved (or fallback mock) data.
    if (gnssConnected) {
      // Retrieve fresh GNSS data using the new getGNSS() function.
      GnssData data = getGNSS();

      // Update saved last fix.
      lastLatitude = data.latitude;
      lastLongitude = data.longitude;
      lastAltitude = data.altitude;

      int32_t lat_int = data.latitude * 1000000; // Convert to integer
      int32_t lon_int = data.longitude * 1000000; // Convert to integer

      // Store latitude in bytes 2-5
      mydata[2] = (lat_int >> 24) & 0xFF;
      mydata[3] = (lat_int >> 16) & 0xFF;
      mydata[4] = (lat_int >> 8) & 0xFF;
      mydata[5] = lat_int & 0xFF;

      // Store longitude in bytes 6-9
      mydata[6] = (lon_int >> 24) & 0xFF;
      mydata[7] = (lon_int >> 16) & 0xFF;
      mydata[8] = (lon_int >> 8) & 0xFF;
      mydata[9] = lon_int & 0xFF;
    }
    else {
      // No new fix: check if a valid previous fix exists.
      if ((lastLatitude != 0.0) || (lastLongitude != 0.0)) {
         int32_t lat_int = lastLatitude * 1000000;
         int32_t lon_int = lastLongitude * 1000000;

         // Store saved latitude in bytes 2-5
         mydata[2] = (lat_int >> 24) & 0xFF;
         mydata[3] = (lat_int >> 16) & 0xFF;
         mydata[4] = (lat_int >> 8) & 0xFF;
         mydata[5] = lat_int & 0xFF;

         // Store saved longitude in bytes 6-9
         mydata[6] = (lon_int >> 24) & 0xFF;
         mydata[7] = (lon_int >> 16) & 0xFF;
         mydata[8] = (lon_int >> 8) & 0xFF;
         mydata[9] = lon_int & 0xFF;
      }
      else {
         // Fallback to mock data (should rarely occur).
         int32_t lat_int = mockLatitude * 1000000;
         int32_t lon_int = mockLongitude * 1000000;

         // Store mock latitude in bytes 2-5
         mydata[2] = (lat_int >> 24) & 0xFF;
         mydata[3] = (lat_int >> 16) & 0xFF;
         mydata[4] = (lat_int >> 8) & 0xFF;
         mydata[5] = lat_int & 0xFF;

         // Store mock longitude in bytes 6-9
         mydata[6] = (lon_int >> 24) & 0xFF;
         mydata[7] = (lon_int >> 16) & 0xFF;
         mydata[8] = (lon_int >> 8) & 0xFF;
         mydata[9] = lon_int & 0xFF;
      }
    }

    // Send data
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    Serial.println(F("Packet queued"));
  }
}

/*============================
   Shake-to-Wake Utilities
=============================*/
void shakeToWakeSetup() {
    if (!lis.begin(0x18)) {   // Change this to 0x19 for an alternative I2C address
       Serial.println("Could not start LIS3DH");
       while(1) yield();
    }
    Serial.println("LIS3DH found!");
    
    lis.setRange(LIS3DH_RANGE_16_G);   // Set to full-scale 16G for lower sensitivity
    Serial.println("Range = 16G");
 
    // Configure click detection with tighter timing
    lis.setClick(1, CLICKTHRESHHOLD, 10, 30, 100);
    delay(100);
 
    // Enable high-pass filtering and interrupt routing/latching 
    enableHPClick();
    enableINT1ClickInterrupt();
    enableINT1Latch();
    Serial.println("Configured INT1 (active low, latched) for tap/doubletap interrupt.");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  // Initialize SPI
  SPI.begin();
  // Print the reason for wakeup
  print_wakeup_reason();

  if (wasNightSleep) {
    wasNightSleep = false;
    customSleepDuration = normalSleepDuration; 
    Serial.print("Woke from night sleep, restoring normal interval: ");
    Serial.println(customSleepDuration);
  }
  // // Blink the LED
  // blinkLED();

  // Disable hold so we can reconfigure the pin
  rtc_gpio_hold_dis((gpio_num_t)LORA_POWER_PIN);

  // Initialize battery gauge
  pinMode(ALR_PIN, INPUT_PULLUP);
  
  pinMode(GPS_POWER_PIN, OUTPUT);    // Set GPS_POWER_PIN as output
  pinMode(LORA_POWER_PIN, OUTPUT);    // Set GPS_POWER_PIN as output

  while(!gnss.begin()){
    Serial.println("No GNSS Device!");
    digitalWrite(GPS_POWER_PIN, HIGH); // Turn the GNSS module on
    delay(1000);
  }

  gnss.enablePower();
  gnss.setGnss(eGPS_GLONASS);
  gnss.setRgbOn();

  attachInterrupt(digitalPinToInterrupt(ALR_PIN), interruptCallBack, FALLING);
  while(gauge.begin() != 0) {
    Serial.println("Gauge initialization failed!");
    delay(2000);
  }
  Serial.println("Gauge initialized successfully!");
  gauge.setInterrupt(BAT_PERCENTAGE);

  // Initialize GNSS module
  digitalWrite(GPS_POWER_PIN, HIGH); // Turn the GNSS module on
  digitalWrite(LORA_POWER_PIN, LOW); // Turn the LORA module on (inverse logic to GNSS)

  #if SHAKE_TO_WAKE_ENABLED
    shakeToWakeSetup();
  #endif
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
}


void loop() {
  switch (currentMode) {
    case OUTDOOR: {
      unsigned long startTime = millis();

      // Attempt to connect with timeout
      while (millis() - startTime < TIMEOUT_DURATION) {
        uint8_t numSatellites = gnss.getNumSatUsed();
        if (numSatellites > 0) {  // Check if GNSS has a satellite fix
          Serial.println("GNSS Connected.");


          // Read GNSS data
          sTim_t utc = gnss.getUTC();
          sTim_t date = gnss.getDate();
          sLonLat_t lat = gnss.getLat();
          sLonLat_t lon = gnss.getLon();
          double high = gnss.getAlt();
          double sog = gnss.getSog();
          double cog = gnss.getCog();

          int TIMEZONE_OFFSET = +1; // or whatever your local offset is from UTC

          // Suppose we want to sleep between 10 PM and 6 AM local time
          int localHour = (utc.hour + TIMEZONE_OFFSET) % 24;
          int localMinute = utc.minute;
          if (localHour >= napTimeHour || localHour < wakeUpHour) {
              Serial.print("It's night time, ");
              Serial.print(localHour);
              Serial.print(":");
              Serial.print(localMinute);
              Serial.println(" (hh:mm), I don't want to work");
              wasNightSleep = true;         // <--- set the flag
              // Convert current time to total minutes since midnight
              int currentTotalMinutes = localHour * 60 + localMinute;

              // Convert wakeUpHour to total minutes (assuming wake at HH:00)
              int targetTotalMinutes = wakeUpHour * 60; 

              // Compute how many minutes until the *next* wakeUpHour,
              // wrapping around 24 hours (1440 minutes) if needed.
              int minutesUntilWake = 
                ( (targetTotalMinutes - currentTotalMinutes) + (24 * 60) ) % (24 * 60);

              // Convert that back to hours and leftover minutes
              int hoursUntilWake   = minutesUntilWake / 60;
              int leftoverMinutes  = minutesUntilWake % 60;

              Serial.print("Want to sleep for another ");
              Serial.print(hoursUntilWake);
              Serial.print(" hours and ");
              Serial.print(leftoverMinutes);
              Serial.println(" minutes.");

              // Convert to whatever time unit your sleep function needs
              // For example, if you need total seconds:
              customSleepDuration = minutesUntilWake * 60;

              // Enter sleep mode
              enterSleepMode();
              // return;  // Don't continue if you decided to sleep
          }

          // Print GNSS data
          Serial.println("");
          Serial.print(date.year); Serial.print("/");
          Serial.print(date.month); Serial.print("/");
          Serial.print(date.date); Serial.print(" ");
          Serial.print(utc.hour); Serial.print(":");
          Serial.print(utc.minute); Serial.print(":");
          Serial.print(utc.second); Serial.println();
          Serial.print("Latitude: "); Serial.println(lat.latitudeDegree, 6);
          Serial.print("Longitude: "); Serial.println(lon.lonitudeDegree, 6);
          Serial.print("Altitude: "); Serial.println(high);
          Serial.print("SOG: "); Serial.println(sog);
          Serial.print("COG: "); Serial.println(cog);
          Serial.print("GNSS mode: "); Serial.println(gnss.getGnssMode());

          delay(1000);  // Delay to print GNSS data regularly
          gnssConnected = true;  // Set the flag to indicate connection is established

          // Schedule data to be sent
          do_send(&sendjob);

          // Loop over os_runloop_once() to allow LMIC to handle transmission
          while (true) {
            os_runloop_once();
            // The loop will exit when enterSleepMode() is called in EV_TXCOMPLETE
          }

          // No break statement needed as we won't reach here
        }
        delay(1000);  // Wait 1 second before trying again
        Serial.print("GPS search duration: ");
        Serial.println(millis() - startTime);
      }

      if (!gnssConnected) {
        if ((lastLatitude != 0.0) || (lastLongitude != 0.0)) {
           Serial.println("No new GNSS fix, using last known location.");
           // Use saved GNSS data to send a packet.
           do_send(&sendjob);
           while (true) {
             os_runloop_once();
           }
        }
        else {
           Serial.println("Connection timed out, no last known fix available, going to sleep.");
           enterSleepMode();
        }
      }
      break;
    }

    case INDOOR: {
      // Simulate GNSS connection and coordinates for indoor mode
      Serial.println("Using mock coordinates as fallback:");
      Serial.print("Mock Latitude: "); Serial.println(mockLatitude, 6);
      Serial.print("Mock Longitude: "); Serial.println(mockLongitude, 6);
      Serial.print("Mock Altitude: "); Serial.println(mockAltitude);

      // Suppose we want to sleep between 10 PM and 6 AM local time
      int localHourMockValue = 20;
      int localMinuteMockValue = 30;
      if (localHourMockValue >= napTimeHour || localHourMockValue < wakeUpHour) {
          Serial.print("It's night time, ");
          Serial.print(localHourMockValue);
          Serial.print(":");
          Serial.print(localMinuteMockValue);
          Serial.println(" (hh:mm), I don't want to work");

          // Convert current time to total minutes since midnight
          int currentTotalMinutes = localHourMockValue * 60 + localMinuteMockValue;

          // Convert wakeUpHour to total minutes (assuming wake at HH:00)
          int targetTotalMinutes = wakeUpHour * 60; 

          // Compute how many minutes until the *next* wakeUpHour,
          // wrapping around 24 hours (1440 minutes) if needed.
          int minutesUntilWake = 
            ( (targetTotalMinutes - currentTotalMinutes) + (24 * 60) ) % (24 * 60);

          // Convert that back to hours and leftover minutes
          int hoursUntilWake   = minutesUntilWake / 60;
          int leftoverMinutes  = minutesUntilWake % 60;

          Serial.print("Want to sleep for another ");
          Serial.print(hoursUntilWake);
          Serial.print(" hours and ");
          Serial.print(leftoverMinutes);
          Serial.println(" minutes.");

          // Convert to whatever time unit your sleep function needs
          // For example, if you need total seconds:
          customSleepDuration = minutesUntilWake * 60;

          // Enter sleep mode
          enterSleepMode();
          // return;  // Don't continue if you decided to sleep
      }

      gnssConnected = false;  // false is needed to send mockup data in do_send
      delay(1000);  // Simulate activity delay

      // Schedule data to be sent
      do_send(&sendjob);

      // Loop over os_runloop_once() to allow LMIC to handle transmission
      while (true) {
        os_runloop_once();
        // The loop will exit when enterSleepMode() is called in EV_TXCOMPLETE
      }
      // No break statement needed as we won't reach here
    }
  }
}

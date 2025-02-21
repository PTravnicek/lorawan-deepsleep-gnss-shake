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
#include "OLEDDisplayController.h"

// Display dimensions and configuration
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define I2C_ADDR 0x3c       // I2C address of the OLED display
#define OLED_RESET -1       // Reset pin (or -1 if sharing Arduino reset pin)
#define OLED_SDA 21         // Custom SDA pin for OLED
#define OLED_SCL 22         // Custom SCL pin for OLED

// Instantiate the OLED display controller
OLEDDisplayController oledController(OLED_SDA, OLED_SCL, I2C_ADDR, SCREEN_WIDTH, SCREEN_HEIGHT, OLED_RESET);

// Debugging option
#define DEBUG 0

#if DEBUG == 1
  #define debug(x)   Serial.print(x)
  #define debugln(x) Serial.println(x)
#else
  #define debug(x)
  #define debugln(x)
#endif

// Define pins
#ifdef __AVR__
  #define ALR_PIN       2
#else
  #define ALR_PIN       D2
#endif

#define LORA_POWER_PIN      D13   // Pin number to control the LDO
#define GPS_POWER_PIN       D11   // Pin number to control the LDO


// Sleep configuration
#define uS_TO_S_FACTOR 1000000ULL // Conversion factor for micro seconds to seconds
#define DEFAULT_SLEEP_TIME 1200
#define TIMEOUT_DURATION 120000   // Timeout duration in milliseconds (2 min)

RTC_DATA_ATTR int customSleepDuration = DEFAULT_SLEEP_TIME; // seconds
RTC_DATA_ATTR bool wasNightSleep = false; 
RTC_DATA_ATTR int  normalSleepDuration = DEFAULT_SLEEP_TIME;

// Battery and GNSS thresholds
#define BAT_PERCENTAGE        15  //bat pin

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
unsigned long fixTime = 0;  // Store the time it took to get a GPS fix

RTC_DATA_ATTR int bootCount = 0;   // Variable to store boot count in RTC memory
int joinRetries = 0;
const int maxJoinRetries = 4; // Set the desired maximum number of retries

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
    case ESP_SLEEP_WAKEUP_EXT0 : debugln("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : debugln("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : debugln("Wakeup caused by timer"); break;
    default : debugln("Wakeup not caused by deep sleep"); break;
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

  // Configure sleep timer
  esp_sleep_enable_timer_wakeup(customSleepDuration * uS_TO_S_FACTOR);

  esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, 1);  // 1 = wake on high level
  debugln("Setup ESP32 to sleep for " + String(customSleepDuration) + " Seconds");
  debugln("Going to sleep now");
  Serial.flush();  
  
  // delay(1000);
  // oledController.powerOn();
  oledController.displayMessage("Sleep (s)", customSleepDuration  );
  delay(1000);
  oledController.powerOff();
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
  debug(os_getTime());
  debug(": ");
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      debugln(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      debugln(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      debugln(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      debugln(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      debugln(F("EV_JOINING"));
      oledController.displayMessage("LoRa", "Joining" );
      break;
    case EV_JOINED:
      debugln(F("EV_JOINED"));
      oledController.displayMessage("LoRa", "Joined" );
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
      debugln(F("EV_JOIN_FAILED"));
      // Go to sleep failed join
      enterSleepMode();
      break;
    case EV_REJOIN_FAILED:
      debugln(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
    debugln(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    oledController.displayMessage("LoRa", "sent" );
      if (LMIC.txrxFlags & TXRX_ACK)
          debugln(F("Received ack"));

      // If there's downlink data, parse it here
      if (LMIC.dataLen) {
          oledController.displayMessage("LoRa", "recived" );
          debug(F("Received "));
          debug(LMIC.dataLen);
          debugln(F(" bytes of payload"));

          // Example: parse the downlink as:
          // Byte[0] = command (e.g., 0x01 = change sleep time, 0x02 = change shake parameters)
          // For command 0x01:
          //    Byte[1..2] = new sleep time in minutes (uint16, big-endian)
          // For command 0x02:
          //    Byte[1] = Shake Enable flag (0: disable, 1: enable)
          //    Byte[2] = New sensitivity threshold for click/shake detection
          uint8_t cmd = LMIC.frame[LMIC.dataBeg + 0];
          if (cmd == 0x01) {
              // Command 0x01: change sleep time.
              uint8_t byteHi = LMIC.frame[LMIC.dataBeg + 1];
              uint8_t byteLo = LMIC.frame[LMIC.dataBeg + 2];
              uint16_t newSleepMinutes = (byteHi << 8) | byteLo;
              customSleepDuration = newSleepMinutes * 60;
              normalSleepDuration = customSleepDuration;  // Store in RTC
              debug("Downlink: Changed normal sleep time to ");
              debugln(customSleepDuration);
              oledController.displayMessage("Sleep", "Change" );
              delay(1000);
              oledController.displayMessage("Sleep", customSleepDuration );
              delay(1000);
          }
          else if (cmd == 0x02) {
              // Command 0x02: change click/shake (LIS3DH) parameters.
              // Byte[1]: Shake Enable flag (0: disable, 1: enable)
              // Byte[2]: New sensitivity threshold (e.g., 10 for default, lower => more sensitive)
              uint8_t shakeEnable = LMIC.frame[LMIC.dataBeg + 1];
              uint8_t newThreshold = LMIC.frame[LMIC.dataBeg + 2];

              if (shakeEnable == 1) {
                  // Enable shake-to-wake (mode=1) with the given sensitivity threshold,
                  // using fixed timing parameters (timelimit=10, timelatency=30, timewindow=100)
                  lis.setClick(1, newThreshold, 10, 30 );
                  debug("Downlink: Shake-to-Wake ENABLED with new sensitivity threshold: ");
                  debugln(newThreshold);
                  oledController.displayMessage("Shake", "threshold" );
                  delay(1000);
                  oledController.displayMessage("Shake", newThreshold );
                  delay(1000);
              } else {
                  // Disable shake-to-wake by setting the click detection mode to 0.
                  lis.setClick(0, 0, 0, 0, 0);
                  debugln("Downlink: Shake-to-Wake DISABLED");
                  oledController.displayMessage("Shake", "change" );
                  delay(1000);
                  oledController.displayMessage("Shake", "disabled" );
                  delay(1000);
              }
          }
          else {
              debug("Downlink command not recognized: 0x");
              Serial.println(cmd, HEX);
          }
      }
      // Go to sleep after transmission
      enterSleepMode();
      break;
    case EV_LOST_TSYNC:
      debugln(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      debugln(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      debugln(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      debugln(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      debugln(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      debugln(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      debugln(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      debugln(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      if (joinRetries < maxJoinRetries) {
        joinRetries++;
        debug("Retrying join attempt ");
        debug(joinRetries);
        debug(" of ");
        debugln(maxJoinRetries);
        oledController.displayMessage("LoRa", "#try" );
        oledController.displayMessage("LoRa", joinRetries );
        LMIC_startJoining();
      } else {
        debugln(F("Maximum join retries reached, going to sleep."));
        oledController.displayMessage("LoRa", "noNetw" );
        enterSleepMode();
      }
      break;
    default:
      debug(F("Unknown event: "));
      debugln((unsigned) ev);
      break;
  }
}

// Function to send data via LoRaWAN
void do_send(osjob_t* j){
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    debugln(F("OP_TXRXPEND, not sending"));
    oledController.displayMessage("LoRa", "busy" );
  } else {
    // Read battery percentage from gauge
    int batteryPercentage = gauge.readPercentage();
    debug("Battery percentage: ");
    debug(batteryPercentage);
    debugln("%");
    oledController.displayMessage("Battery", batteryPercentage );
    delay(1000);

    if(intFlag == 1) {
      intFlag = 0;
      gauge.clearInterrupt();
      debugln("Low power alert interrupt!");
      // Handle low battery alert here
    }

    // Prepare data to send with payload size 11 bytes:
    // Byte 0: Battery percentage
    // Byte 1: GPS fix time in seconds
    // Byte 2: Wake-up reason (esp_sleep_get_wakeup_cause())
    // Bytes 3-6: Latitude (32-bit integer)
    // Bytes 7-10: Longitude (32-bit integer)
    uint8_t mydata[11];
    mydata[0] = batteryPercentage;
    mydata[1] = (uint8_t)((fixTime + 500) / 1000);  // Round to nearest second
    mydata[2] = (uint8_t) esp_sleep_get_wakeup_cause();

    if (gnssConnected) {
      // Retrieve fresh GNSS data using the new getGNSS() function.
      oledController.displayMessage("GPS", "fix OK" );
      GnssData data = getGNSS();

      // Update saved last fix.
      lastLatitude = data.latitude;
      lastLongitude = data.longitude;
      lastAltitude = data.altitude;

      int32_t lat_int = data.latitude * 1000000; // Convert to integer
      int32_t lon_int = data.longitude * 1000000; // Convert to integer

      // Store latitude in bytes 3-6
      mydata[3] = (lat_int >> 24) & 0xFF;
      mydata[4] = (lat_int >> 16) & 0xFF;
      mydata[5] = (lat_int >> 8) & 0xFF;
      mydata[6] = lat_int & 0xFF;

      // Store longitude in bytes 7-10
      mydata[7] = (lon_int >> 24) & 0xFF;
      mydata[8] = (lon_int >> 16) & 0xFF;
      mydata[9] = (lon_int >> 8) & 0xFF;
      mydata[10] = lon_int & 0xFF;
    }
    else {
      // No new fix: check if a valid previous fix exists.
      oledController.displayMessage("GPS", "no fix" );
      delay(1000);
      if ((lastLatitude != 0.0) || (lastLongitude != 0.0)) {
        oledController.displayMessage("GPS", "last pos" );
        delay(1000);
        int32_t lat_int = lastLatitude * 1000000;
        int32_t lon_int = lastLongitude * 1000000;

        // Store saved latitude in bytes 3-6
        mydata[3] = (lat_int >> 24) & 0xFF;
        mydata[4] = (lat_int >> 16) & 0xFF;
        mydata[5] = (lat_int >> 8) & 0xFF;
        mydata[6] = lat_int & 0xFF;

        // Store saved longitude in bytes 7-10
        mydata[7] = (lon_int >> 24) & 0xFF;
        mydata[8] = (lon_int >> 16) & 0xFF;
        mydata[9] = (lon_int >> 8) & 0xFF;
        mydata[10] = lon_int & 0xFF;
      }
      else {
        // Fallback to mock data (should rarely occur).
        oledController.displayMessage("GPS", "mock pos" );
        int32_t lat_int = mockLatitude * 1000000;
        int32_t lon_int = mockLongitude * 1000000;

        // Store mock latitude in bytes 3-6
        mydata[3] = (lat_int >> 24) & 0xFF;
        mydata[4] = (lat_int >> 16) & 0xFF;
        mydata[5] = (lat_int >> 8) & 0xFF;
        mydata[6] = lat_int & 0xFF;

        // Store mock longitude in bytes 7-10
        mydata[7] = (lon_int >> 24) & 0xFF;
        mydata[8] = (lon_int >> 16) & 0xFF;
        mydata[9] = (lon_int >> 8) & 0xFF;
        mydata[10] = lon_int & 0xFF;
      }
    }

    // Send data
    oledController.displayMessage("LoRa", "sending..." );
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
    debugln(F("Packet queued"));
  }
}

/*============================
   Shake-to-Wake Utilities
=============================*/
void shakeToWakeSetup() {
    if (!lis.begin(0x18)) {   // Change this to 0x19 for an alternative I2C address
      oledController.displayMessage("Shake", "err" );
       debugln("Could not start LIS3DH");
       while(1) yield();
    }
    oledController.displayMessage("Shake", "ok" );
    debugln("LIS3DH found!");
    
    lis.setRange(LIS3DH_RANGE_16_G);   // Set to full-scale 16G for lower sensitivity
    debugln("Range = 16G");
 
    // Configure click detection with tighter timing
    lis.setClick(1, CLICKTHRESHHOLD, 10, 30 );
    delay(100);
 
    // Enable high-pass filtering and interrupt routing/latching 
    enableHPClick();
    enableINT1ClickInterrupt();
    enableINT1Latch();
    debugln("Configured INT1 (active low, latched) for tap/doubletap interrupt.");
}


// Calculate sleep duration until wake-up time
int calculateSleepDuration(int currentHour, int currentMinute) {
    int currentTotalMinutes = currentHour * 60 + currentMinute;
    int targetTotalMinutes = wakeUpHour * 60;
    return ((targetTotalMinutes - currentTotalMinutes) + (24 * 60)) % (24 * 60);
}

// Function to check if it's night time and handle sleep if needed
bool checkNightTime(int hour, int minute) {
    int localHour = (hour + 1) % 24;  // TIMEZONE_OFFSET = +1
    
    if (localHour >= napTimeHour || localHour < wakeUpHour) {
        int minutesUntilWake = calculateSleepDuration(localHour, minute);
        customSleepDuration = minutesUntilWake * 60;
        wasNightSleep = true;
        enterSleepMode();
        return true;
    }
    return false;
}

// Function to handle GNSS operations and return status
bool handleGNSS() {
    unsigned long startTime = millis();
    debugln("GNSS Searching.");
    oledController.displayMessage("GPS", "search" );

    // Attempt to connect with timeout
    while (millis() - startTime < TIMEOUT_DURATION) {
        uint8_t numSatellites = gnss.getNumSatUsed();
        if (numSatellites > 0) {
            debugln("GNSS Connected.");
            fixTime = millis() - startTime;  // Update global fixTime
            debug("GNSS fix obtained in ");
            debug(fixTime);
            debugln(" ms");
            // Read GNSS data
            sTim_t utc = gnss.getUTC();
            sTim_t date = gnss.getDate();
            sLonLat_t lat = gnss.getLat();
            sLonLat_t lon = gnss.getLon();
            
            // Update last known position
            lastLatitude = lat.latitudeDegree;
            lastLongitude = lon.lonitudeDegree;
            lastAltitude = gnss.getAlt();

            // Check for night-time sleep
            if (checkNightTime(utc.hour, utc.minute)) {
                return false;
            }

            return true;
        }
        delay(1000);
        debug("GPS fixing time (ms): ");
        debugln(millis() - startTime);
    }
    return false;
}




void setup() {  
  // Disable WiFi and Bluetooth
  // WiFi.mode(WIFI_OFF);
  // btStop();
  
  // // Disable unused ADC to save power
  // adc_power_off();

  Serial.begin(115200);
  // Disable hold so we can reconfigure the pin
  rtc_gpio_hold_dis((gpio_num_t)LORA_POWER_PIN);

  pinMode(GPS_POWER_PIN, OUTPUT);    // Set GPS_POWER_PIN as output
  digitalWrite(GPS_POWER_PIN, HIGH); // Turn the GNSS module on

  pinMode(LORA_POWER_PIN, OUTPUT);    // Set GPS_POWER_PIN as output
  digitalWrite(LORA_POWER_PIN, LOW); // Turn the LORA module on (inverse logic to GNSS)

  // Initialize SPI
  SPI.begin();
  oledController.begin();
  delay(1000);
  ++bootCount;
  debugln("Boot number: " + String(bootCount));

  // Print the reason for wakeup
  print_wakeup_reason();
  
  oledController.powerOn();
  oledController.displayMessage("Setup", "..." );

  // Initialize battery gauge
  pinMode(ALR_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(ALR_PIN), interruptCallBack, FALLING);
  while(gauge.begin() != 0) {
    debugln("Gauge initialization failed!");
    delay(2000);
  }
  debugln("Gauge initialized successfully!");
  // gauge.setInterrupt(BAT_PERCENTAGE);
  int batteryPercentage = gauge.readPercentage();
  debug("Battery percentage setup: ");
  debug(batteryPercentage);
  debugln("%");

  if (wasNightSleep) {
    wasNightSleep = false;
    customSleepDuration = normalSleepDuration; 
    debug("Woke from night sleep, restoring normal interval: ");
    debugln(customSleepDuration);
  }

  // oledController.displayMessage("GPS", "begin" );
  while(!gnss.begin()){
    debugln("No GNSS Device!");
    digitalWrite(GPS_POWER_PIN, HIGH); // Turn the GNSS module on
    delay(1000);
  }
  oledController.displayMessage("GPS", "ok" );

  gnss.enablePower();
  gnss.setGnss(eGPS_GLONASS);
  gnss.setRgbOff();

  #if SHAKE_TO_WAKE_ENABLED
    shakeToWakeSetup();
  #endif
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  oledController.displayMessage("Setup", "finished" );
  // oledController.powerOff();
}


void loop() {
  switch (currentMode) {
    case OUTDOOR: {
      if (handleGNSS()) {
          do_send(&sendjob);
          while (!(LMIC.opmode & OP_TXRXPEND)) {
              esp_light_sleep_start();
              os_runloop_once();
          }
      } else if ((lastLatitude != 0.0) || (lastLongitude != 0.0)) {
          digitalWrite(GPS_POWER_PIN, LOW);
          do_send(&sendjob);
          while (true) {
              os_runloop_once();
          }
      } else {
          enterSleepMode();
      }
      break;
    }

    case INDOOR: {
      digitalWrite(GPS_POWER_PIN, LOW);    // turn the GNSS module OFF
      // oledController.displayMessage("GPS", "indoor" );
      // oledController.displayMessage("GPS", "off" );
      // Simulate GNSS connection and coordinates for indoor mode

      debugln("Using mock coordinates as fallback:");
      debug("Mock Latitude: "); Serial.println(mockLatitude, 6);
      debug("Mock Longitude: "); Serial.println(mockLongitude, 6);
      debug("Mock Altitude: "); Serial.println(mockAltitude);

      // Suppose we want to sleep between 10 PM and 6 AM local time
      int localHourMockValue = 20;
      int localMinuteMockValue = 30;
      if (localHourMockValue >= napTimeHour || localHourMockValue < wakeUpHour) {
          // oledController.displayMessage("Night", "time" );
          debug("It's night time, ");
          debug(localHourMockValue);
          debug(":");
          debug(localMinuteMockValue);
          debugln(" (hh:mm), I don't want to work");

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

          debug("Want to sleep for another ");
          debug(hoursUntilWake);
          debug(" hours and ");
          debug(leftoverMinutes);
          debugln(" minutes.");

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

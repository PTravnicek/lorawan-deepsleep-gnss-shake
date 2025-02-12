# LoRaWAN Deep Sleep Device with GNSS & Shake-to-Wake

This project demonstrates a low-power LoRaWAN device based on the ESP32 that integrates several features:
- **GNSS (GPS/GLONASS)** for acquiring location data.
- **Battery Gauge** monitoring using a DFRobot_MAX17043.
- **Shake-to-Wake** functionality using the Adafruit LIS3DH accelerometer.
- **LoRaWAN** communication using the LMIC library for over-the-air activation (OTAA) and data transmission.
- **Deep Sleep Management** where the device sleeps to conserve power and wakes upon specific events (timer or shake).

The transmitted payload includes:
- Battery percentage.
- Wake-up reason (from ESP32 deep sleep wake-up cause).
- GNSS coordinates (latitude and longitude in microdegrees).

A companion Node-RED flow (in `node-red/return_to_influxDB.js`) decodes the 10-byte payload and writes the data into InfluxDB for monitoring and visualization.

---

## Features

- **LoRaWAN Communication:**  
  Uses the LMIC stack with OTAA configuration to join a LoRaWAN network and send sensor data.
  
- **GNSS Data Collection:**  
  Retrieves GNSS coordinates using a GNSS module. The device sends current coordinates if available, or falls back to saved (RTC memory) or mock data when no fix is obtained.
  
- **Low-Power Operation:**  
  The device enters deep sleep between transmissions to conserve energy. RTC memory retains state (e.g., the last known GNSS fix and sleep duration) over sleep cycles.

- **Shake-to-Wake Capability:**  
  An Adafruit LIS3DH accelerometer monitors for tap/double-tap events. When a strong shake is detected, the sensor wakes the ESP32 from deep sleep. This feature can be enabled or disabled at compile time.
  
- **Battery Monitoring:**  
  A battery gauge monitors battery percentage and triggers a low-power alert if below a set threshold.

- **Configurable Sleep Interval:**  
  The device adjusts sleep intervals based on time-of-day (for example, longer sleep during night time).

---

## Hardware Requirements

- **ESP32 Development Board**
- **LoRa Transceiver Module** (configured via LMIC)
- **GNSS Module:** Compatible with the DFRobot_GNSS library
- **Battery Gauge:** DFRobot_MAX17043 or similar
- **Accelerometer:** Adafruit LIS3DH connected via I2C for shake-to-wake
- **Additional Components:**  
  - LEDs, wires, and appropriate power management circuitry
  - Custom wiring for INT pins and power control of GNSS/LoRa modules

### Pin Configuration

Make sure to review and adjust the following pins in `src/main.cpp`:
- `GPS_POWER_PIN`: Controls power to the GNSS module.
- `LORA_POWER_PIN`: Controls power to the LoRa module.
- `WAKEUP_PIN`: Connected to the accelerometer’s INT1 pin.
- `ALR_PIN`: For the battery gauge interrupt.

---

## Software Requirements

- **Arduino IDE** or PlatformIO configured for the ESP32.
- Required libraries:
  - [LMIC](https://github.com/mcci-catena/arduino-lmic)
  - [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
  - [DFRobot_MAX17043](https://github.com/DFRobot/DFRobot_MAX17043)
  - [DFRobot_GNSS](https://github.com/DFRobot/DFRobot_GNSS)
  - [Adafruit LIS3DH](https://github.com/adafruit/Adafruit_LIS3DH)
  - [Adafruit Sensor](https://github.com/adafruit/Adafruit_Sensor)
- Node-RED with an InfluxDB output to process the payload. See the provided `node-red/return_to_influxDB.js` for payload decoding.

---

## Getting Started

### 1. Clone the repository (not on the repository yet)
git clone https://github.com/your_username/lorawan-deepsleep-gnss-shake.git


### 2. Configure LoRaWAN Credentials

Edit `src/main.cpp` and update the keys for OTAA:
- `APPEUI`
- `DEVEUI`
- `APPKEY`

### 3. Hardware Setup

- Connect the GNSS module, battery gauge, and LIS3DH sensor to the ESP32 per the pin assignments.
- Verify wiring for INT pin from the accelerometer.
- Make sure the power control pins (`GPS_POWER_PIN` and `LORA_POWER_PIN`) are correctly connected.

### 4. Customize Features

- **Shake-to-Wake:**  
  To disable or enable shake-to-wake, modify the macro in `src/main.cpp`:
  ```cpp
  #define SHAKE_TO_WAKE_ENABLED true  // Set to false to disable
  ```
- **Sleep Duration:**  
  The sleep interval defaults to 1200 seconds (20 minutes) but can be modified via `customSleepDuration` or via downlink commands.

### 5. Build and Upload

Compile and flash the firmware using your preferred IDE (Arduino IDE or PlatformIO).

### 6. Node-RED Integration

- Deploy the provided Node-RED flow in the `node-red/` folder.
- The flow decodes the 10-byte payload. Byte 0 is battery percentage, byte 1 is wake-up reason, bytes 2-5 are latitude, and bytes 6-9 are longitude.
- Configure InfluxDB connections as needed.

---

## Project Structure
├── src
│ └── main.cpp // Main application code for the ESP32
├── node-red
│ └── return_to_influxDB.js // Node-RED flow to decode payloads and forward to InfluxDB
├── README.md // Project documentation (this file)
└── LICENSE // License file (if applicable)


---

## How It Works

1. **Deep Sleep Cycle:**
   - The device initializes various modules (GNSS, battery gauge, accelerometer).
   - It checks for any GNSS satellite fix. If not available, it falls back to saved coordinates.
   - The device then transmits a LoRaWAN packet containing battery, wake-up reason, and location data.
   - After transmission, it enters deep sleep to conserve power.

2. **Wake-Up Sources:**
   - **Timer Wake-Up:** The device wakes after a pre-configured sleep duration.
   - **External (Shake) Wake-Up:** If enabled, a shake (tap/double-tap) event detected by the LIS3DH wakes the device from deep sleep.

3. **Payload Format:**
   - The first byte contains the battery percentage.
   - The second byte records the wake-up reason (as provided by the ESP32’s wake-up cause).
   - The next four bytes provide the latitude in microdegrees.
   - The final four bytes give the longitude in microdegrees.

---

## License

This project is released under the [MIT License](LICENSE).

---

## Acknowledgments

- Thanks to the developers of the LMIC library for LoRaWAN support.
- Thanks to the Arduino and ESP32 communities for excellent support and documentation.
- Special thanks to Adafruit for sensor libraries and hardware designs.


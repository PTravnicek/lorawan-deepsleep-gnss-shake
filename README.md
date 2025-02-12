# LoRaWAN Deep Sleep Device with GNSS & Shake-to-Wake

This project demonstrates a low-power LoRaWAN device based on the ESP32 that integrates several features:
- **GNSS (GPS/GLONASS):** Retrieves location data with fallback to previously stored or mock data if a new fix is not available.
- **Battery Gauge:** Monitors battery voltage/percentage using a DFRobot_MAX17043.
- **Shake-to-Wake:** Uses an Adafruit LIS3DH accelerometer to detect tap/double-tap events to wake the device from deep sleep. This functionality can be enabled or disabled via firmware settings or updated remotely using downlink commands.
- **LoRaWAN Communication:** Uses the LMIC library for OTAA to join a LoRaWAN network and transmit sensor and wake-up data.
- **Deep Sleep Management:** The device conserves energy by entering deep sleep and wakes either on a timer or an external event (such as a shake).

The transmitted payload is 10 bytes, with the following layout:
- **Byte 0:** Battery percentage
- **Byte 1:** Wake-up reason (numeric value as returned by `esp_sleep_get_wakeup_cause()`)
- **Bytes 2–5:** Latitude (signed 32-bit integer, microdegrees)
- **Bytes 6–9:** Longitude (signed 32-bit integer, microdegrees)

A companion Node-RED flow in `node-red/return_to_influxDB.js` decodes the 10-byte payload and writes the data into InfluxDB. It now includes both the numeric wake-up reason and a human-readable string.

---

## Features

- **LoRaWAN Communication:**  
  Uses the LMIC stack with OTAA configuration to join and maintain a LoRaWAN session.

- **GNSS Data Collection:**  
  Retrieves current GNSS coordinates when available. If a new fix is not found, the device falls back to a previously stored coordinate (in RTC memory) or mock data.

- **Low-Power Operation:**  
  The device enters deep sleep between transmissions. RTC memory is used to preserve key state information such as the last GNSS fix, sleep interval, and boot count.

- **Shake-to-Wake Capability:**  
  The Adafruit LIS3DH accelerometer is used to detect shake/tap events. The functionality can be toggled (ON/OFF) and the sensitivity can be adjusted remotely using downlink commands.

- **Battery Monitoring:**  
  The device monitors battery levels with the DFRobot_MAX17043 battery gauge and includes battery percentage in the transmitted payload.

- **Remote Configuration (Downlink):**  
  The device can be remotely reconfigured using downlink messages. Two commands are supported:
  - **Command 0x01:** Update the normal sleep time.
  - **Command 0x02:** Change click/shake parameters (enable/disable shake-to-wake and update the sensitivity threshold).

- **Node-RED Integration:**  
  A Node-RED flow decodes the incoming payload, which now also includes a human-readable wake-up reason, and writes the data to InfluxDB for visualization.

---

## Hardware Requirements

- **ESP32 Development Board**
- **LoRa Transceiver Module:** Configured using LMIC.
- **GNSS Module:** Compatible with the DFRobot_GNSS library.
- **Battery Gauge:** DFRobot_MAX17043 or similar.
- **Accelerometer:** Adafruit LIS3DH (I2C connection) for shake-to-wake.
- **Additional Components:**  
  - LEDs, wires, and power management circuitry.
  - Custom wiring for INT pins and power control (e.g., `GPS_POWER_PIN` and `LORA_POWER_PIN`).

### Pin Configuration

In `src/main.cpp`, verify and adjust:
- `GPS_POWER_PIN`: Controls the GNSS module's power.
- `LORA_POWER_PIN`: Controls the LoRa module's power.
- `WAKEUP_PIN`: Connected to the accelerometer's INT1 output.
- `ALR_PIN`: For the battery gauge interrupt.

---

## Software Requirements

- **Arduino IDE** or PlatformIO for ESP32 development.
- Required libraries:
  - [LMIC](https://github.com/mcci-catena/arduino-lmic)
  - [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
  - [DFRobot_MAX17043](https://github.com/DFRobot/DFRobot_MAX17043)
  - [DFRobot_GNSS](https://github.com/DFRobot/DFRobot_GNSS)
  - [Adafruit LIS3DH](https://github.com/adafruit/Adafruit_LIS3DH)
  - [Adafruit Sensor](https://github.com/adafruit/Adafruit_Sensor)
- **Node-RED:**  
  With an InfluxDB output node. See `node-red/return_to_influxDB.js` for payload decoding.

---

## Getting Started

### 1. Clone the Repository

If you haven't already, clone the project repository:

```bash
git clone https://github.com/PTravnicek/lorawan-deepsleep-gnss-shake.git
```

### 2. Configure LoRaWAN Credentials

Edit `src/main.cpp` to update the following keys for OTAA:
- `APPEUI`
- `DEVEUI`
- `APPKEY`

### 3. Hardware Setup

- Connect the GNSS module, battery gauge, and LIS3DH sensor to the ESP32.
- Verify wiring, especially the INT pin connection on the accelerometer.
- Ensure power control pins for the GNSS and LoRa modules (`GPS_POWER_PIN` and `LORA_POWER_PIN`) are correctly configured.

### 4. Customize Features

- **Shake-to-Wake:**  
  You can enable or disable the shake-to-wake feature using the macro defined in `src/main.cpp`:
  ```cpp
  #define SHAKE_TO_WAKE_ENABLED true  // Set to false to disable
  ```
- **Sleep Duration:**  
  The default sleep duration is set to 1200 seconds (20 minutes). It can be changed via downlink command 0x01 or by modifying the `customSleepDuration` variable directly.

### 5. Build and Upload

Compile and upload the firmware to your ESP32 using your preferred environment (Arduino IDE, PlatformIO, etc.).

### 6. Node-RED Integration

- Deploy the Node-RED flow from the `node-red/` folder.
- The flow decodes the 10-byte payload:
  - **Byte 0:** Battery percentage.
  - **Byte 1:** Wake-up reason (numeric) and also a human-readable field (`wakeUpReasonText`).
  - **Bytes 2–5:** Latitude.
  - **Bytes 6–9:** Longitude.
- Configure the InfluxDB connection as needed.

---

## Downlink Command Examples

The device supports remote configuration via downlink messages. The following commands can be sent from your LoRaWAN network server:

### Command 0x01: Change Sleep Time

- **Format:**  
  - **Byte 0:** `0x01`
  - **Bytes 1–2:** New sleep time in minutes (16-bit big-endian).
  
- **Example:**  
  To set the sleep time to 20 minutes, send:
  ```
  01 00 14
  ```
  (Here, `0x14` in hexadecimal is `20` in decimal.)

### Command 0x02: Change Click/Shake (LIS3DH) Parameters

- **Format:**  
  - **Byte 0:** `0x02`
  - **Byte 1:** Shake Enable Flag (`0` to disable, `1` to enable).
  - **Byte 2:** New sensitivity threshold (lower value means more sensitive; default is typically `10`).
  
- **Examples:**  
  - **Enable Shake-to-Wake with Sensitivity Threshold of 15:**  
    ```
    02 01 0F
    ```
    (Shake enabled with threshold `0x0F` = 15.)
    
  - **Disable Shake-to-Wake:**  
    ```
    02 00 00
    ```

These downlink messages allow you to remotely fine-tune the device operation without needing a physical connection.

---

## Project Structure

```
.
├── src
│   └── main.cpp          // Main application code for the ESP32
├── node-red
│   └── return_to_influxDB.js  // Node-RED flow for payload decoding and InfluxDB integration
├── README.md             // Project documentation (this file)
└── LICENSE               // License file (if applicable)
```

---

## How It Works

1. **Deep Sleep & Wake-Up Cycle:**
   - On boot, the device initializes the GNSS, battery gauge, and (if enabled) the accelerometer for shake-to-wake.
   - The device attempts to acquire a new GNSS fix. If not available, it uses the last stored fix from RTC memory.
   - A LoRaWAN packet is formed, containing battery percentage, wake-up cause, and GNSS coordinates.
   - The device then enters deep sleep to conserve power.

2. **Wake-Up Sources:**
   - **Timer Wake-Up:** Wakes after a pre-configured sleep interval.
   - **External (Shake/Tap):** When enabled, shake events detected by the LIS3DH wake the device immediately.

3. **Downlink Commands:**
   - The device listens for downlink messages after a transmission.
   - Depending on the command received, it can update its sleep interval or modify shake detection parameters.
   - Changes are applied immediately and stored in RTC memory for persistence across sleeps.

4. **Payload Format:**
   - **Battery Percentage:** Indicates current battery status.
   - **Wake-Up Reason:** Provided as both a numeric value and a human-readable string.
   - **GNSS Coordinates:** Latitude and longitude in microdegrees for precise location reporting.

---

## License

This project is released under the [MIT License](LICENSE).

---

## Acknowledgments

- Thanks to the developers of the LMIC library for LoRaWAN support.
- Appreciation is extended to the Arduino and ESP32 communities for their support and documentation.
- Special mentions to Adafruit for sensor libraries and hardware designs.

---

## Contact

If you have any questions or suggestions, please feel free to [open an issue](https://github.com/PTravnicek/lorawan-deepsleep-gnss-shake/issues) or contact [your_email@example.com](mailto:pavel.travnicek@fsv.cvut.cz).


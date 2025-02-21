#ifndef OLED_DISPLAY_CONTROLLER_H
#define OLED_DISPLAY_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

class OLEDDisplayController {
public:
    // Constructor
    OLEDDisplayController(uint8_t sda, uint8_t scl, uint8_t i2c_addr,
                          int16_t screen_width, int16_t screen_height, int8_t reset_pin);
    ~OLEDDisplayController();

    // Initializes the custom I2C bus and display
    void begin();

    // Removed displayTime parameter
    void displayMessage(const char* functionName, const char* status, uint8_t brightness = 10);
    void displayMessage(const char* functionName, int status, uint8_t brightness = 10);

    // Turn on the display (send command 0xAF)
    void powerOn();

    // Turn off the display (send command 0xAE)
    void powerOff();

    // Provide access to the underlying display object
    Adafruit_SH1106G* getDisplay();

private:
    TwoWire* _wire;
    Adafruit_SH1106G* _display;
    uint8_t  _sda;
    uint8_t  _scl;
    uint8_t  _i2c_addr;
    int16_t  _screen_width;
    int16_t  _screen_height;
    int8_t   _reset_pin;
};

#endif // OLED_DISPLAY_CONTROLLER_H 
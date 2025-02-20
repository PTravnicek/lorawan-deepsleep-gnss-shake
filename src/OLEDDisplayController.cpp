#include "OLEDDisplayController.h"

OLEDDisplayController::OLEDDisplayController(uint8_t sda, uint8_t scl, uint8_t i2c_addr,
                                             int16_t screen_width, int16_t screen_height, int8_t reset_pin)
    : _sda(sda), _scl(scl), _i2c_addr(i2c_addr),
      _screen_width(screen_width), _screen_height(screen_height), _reset_pin(reset_pin)
{
    // // Create a dedicated I2C instance on bus 0 for the OLED display.
    // _wire = new TwoWire(1);
    // I2C instance on bus 0 for the OLED display.
    _wire = new TwoWire(1);
    _display = new Adafruit_SH1106G(_screen_width, _screen_height, _wire, _reset_pin);
}

OLEDDisplayController::~OLEDDisplayController() {
    if (_display) {
        delete _display;
    }
    if (_wire) {
        delete _wire;
    }
}

void OLEDDisplayController::begin() {
    // Initialize the OLED I2C bus with default frequency
    _wire->begin(_sda, _scl);
    _display->begin(_i2c_addr, true);
    _display->clearDisplay();
}

void OLEDDisplayController::displayMessage(const char* functionName, const char* status, uint8_t brightness) {
    _display->clearDisplay();
    
    _display->setTextSize(1.5);
    _display->setTextColor(SH110X_WHITE);
    _display->setContrast(brightness);
    
    int16_t x, y;
    uint16_t w, h;
    // Center the functionName text
    _display->getTextBounds(functionName, 0, 0, &x, &y, &w, &h);
    int funcX = (_screen_width - w) / 2;
    int funcY = (_screen_height - 2 * h) / 2;
    _display->setCursor(funcX, funcY);
    _display->println(functionName);
    
    // Center the status text below the first line
    _display->getTextBounds(status, 0, 0, &x, &y, &w, &h);
    int statusX = (_screen_width - w) / 2;
    int statusY = funcY + h;
    _display->setCursor(statusX, statusY);
    _display->println(status);
    
    _display->display();
}

// New function: Turns on the display by sending command 0xAF
void OLEDDisplayController::powerOn() {
    _wire->beginTransmission(_i2c_addr);
    _wire->write(0x00); // Command mode
    _wire->write(0xAF); // Display on command
    _wire->endTransmission();
}

// New function: Turns off the display by sending command 0xAE
void OLEDDisplayController::powerOff() {
    _display->clearDisplay();
    _display->display();  // Add this to actually show the blank screen
    _wire->beginTransmission(_i2c_addr);
    _wire->write(0x00); // Command mode
    _wire->write(0xAE); // Display off command
    _wire->endTransmission();
}

// Update the integer overload
void OLEDDisplayController::displayMessage(const char* functionName, int status, uint8_t brightness) {
    char statusStr[16];
    itoa(status, statusStr, 10);
    displayMessage(functionName, statusStr, brightness);
}

Adafruit_SH1106G* OLEDDisplayController::getDisplay() {
    return _display;
} 
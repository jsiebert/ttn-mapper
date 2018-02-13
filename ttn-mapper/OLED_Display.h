/*
 * oled_display.h - Helper class for Adafruit OLED Featherwing
 * 
 * The OLED FeatherWing is a 128x32 pixels display
 * Characters are 6x8 pixels -- 4 lines of 21 chars
 * 
 */

#ifndef _oled_display_h
#define _oled_display_h

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <lmic.h>

// TTN Logo, 72x32px
// Seems that we need byte alignement...
const uint8_t ttn_logo_w = 72;
const uint8_t ttn_logo_h = 32;
const uint8_t ttn_logo [] = {
  0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xc0, 0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 
  0xe0, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xf0, 0x01, 0xfc, 0x00, 0x00, 0x00, 0x78, 
  0x3f, 0xff, 0xf8, 0x00, 0x7e, 0x00, 0x00, 0x01, 0xfe, 0x7f, 0xff, 0xf8, 0x01, 0x9f, 0x00, 0x00, 
  0x03, 0xff, 0x7f, 0xff, 0xfc, 0x03, 0xef, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xfc, 0xf3, 0xf7, 
  0x80, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfb, 0x80, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0x3b, 0xc0, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3d, 0xc0, 0x00, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0x9d, 0xc0, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdd, 0xc0, 0x01, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xdd, 0xc0, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xdd, 0xc0, 
  0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xd9, 0xc0, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xc1, 0x80, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0x80, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 
  0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 
  0xff, 0xf0, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x3f, 0xff, 0xff, 0xff, 
  0xff, 0xff, 0xff, 0xe0, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x0f, 0xff, 
  0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00
};

// Display properties
const uint8_t oled_w = 128;
const uint8_t oled_h = 32;
const uint8_t oled_char_w = 6;
const uint8_t oled_char_h = 8;

// Battery voltage pin
// Warning: Analog A7 is also Digital #9 which is "Button A" on the OLED FeatherWing.
// If you plan to use this button, do not connect "A" to #9 and solder a bridge from
// "A" to an unused pin (e.g. #10) on the FeatherWing.
const uint8_t battery_pin = A7;

// Class definition
class OLED_Display: public Adafruit_SSD1306
{
  protected:
    float _battery;
    void _readBatteryVotlatge( void );
    s4_t _uptime( void );
    uint32_t _sent;
    uint32_t _error;
    uint32_t _complete;
    
  public:
    // Constructor
    OLED_Display ( int reset = -1 ) : Adafruit_SSD1306(reset)
    {
      _battery = 0.0;
      _sent = 0;
      _error = 0;
      _complete = 0;
    }
    
    void init ( void );
    void clearLine ( uint8_t line );
    void clearText ( void );
    void update ( void );
    String hms(ostime_t t);
    void addSent ( void );
    void addError ( void );
    void addComplete ( void );
    float getBatteryVoltage ( void );
};
 

#endif /* _oled_display_h */

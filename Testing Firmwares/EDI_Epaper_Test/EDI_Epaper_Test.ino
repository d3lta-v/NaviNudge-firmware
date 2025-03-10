/***************************************************
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_EPD.h"
#include <Adafruit_GFX.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;

#define EPD_DC 14
#define EPD_CS 12
#define EPD_BUSY 26 // can set to -1 to not use a pin (will wait a fixed delay)
#define SRAM_CS 13
#define EPD_RESET 27  // can set to -1 and share with microcontroller Reset!
#define EPD_SPI &SPI // primary SPI

// Uncomment the following line if you are using 1.54" EPD with IL0373
// Adafruit_IL0373 display(152, 152, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);
// Uncomment the following line if you are using 1.54" EPD with SSD1680
// Adafruit_SSD1680 display(152, 152, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);
// Uncomment the following line if you are using 1.54" EPD with SSD1608
// Adafruit_SSD1608 display(200, 200, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);
// Uncomment the following line if you are using 1.54" EPD with SSD1681
// Adafruit_SSD1681 display(200, 200, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);
// Uncomment the following line if you are using 1.54" EPD with UC8151D
// Adafruit_UC8151D display(152, 152, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.13" EPD with SSD1680
// Adafruit_SSD1680 display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.13" EPD with SSD1675
// Adafruit_SSD1675 display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.13" EPD with SSD1675B
// Adafruit_SSD1675B display(250, 122, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.13" EPD with UC8151D
// Adafruit_UC8151D display(212, 104, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.13" EPD with IL0373
// Adafruit_IL0373 display(212, 104, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY,
//                         EPD_SPI);
//#define FLEXIBLE_213

// Uncomment the following line if you are using 2.7" EPD with IL91874
// Adafruit_IL91874 display(264, 176, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.7" EPD with EK79686
// Adafruit_EK79686 display(264, 176, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.9" EPD with IL0373
// Adafruit_IL0373 display(296, 128, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI); 
// #define FLEXIBLE_290

// Uncomment the following line if you are using 2.9" EPD with SSD1680
// Adafruit_SSD1680 display(296, 128, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS,
// EPD_BUSY, EPD_SPI);

// Uncomment the following line if you are using 2.9" EPD with UC8151D
Adafruit_UC8151D display(296, 128, EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);
// #define FLEXIBLE_290

#define COLOR1 EPD_BLACK
#define COLOR2 EPD_RED

void setup() {
  Serial.begin(115200);
  // while (!Serial) { delay(10); }

  if (! drv.begin()) {
    Serial.println("Could not find DRV2605");
    while (1) delay(10);
  } else {
    drv.selectLibrary(6);
    drv.setMode(DRV2605_MODE_INTTRIG); 
    drv.useLRA();
  }

  display.begin();

  Serial.println("Display began");

#if defined(FLEXIBLE_213) || defined(FLEXIBLE_290)
  // The flexible displays have different buffers and invert settings!
  display.setBlackBuffer(1, false);
  display.setColorBuffer(1, false);
  Serial.println("Colour buffer modified for flex displays");
#endif

  // large block of text
  display.setRotation(1);
  display.clearBuffer();
  testdrawtext(
      "NaviNudge\n"
      "v0.1",
      COLOR1);
  // display direction
  display.fillTriangle(30, 95, 128-30, 95, 64, 50, COLOR1);
  display.fillRect(49, 95, 32, 130, COLOR1);
  display.display();
  Serial.println("Text printed");

  delay(5000);

  // display.clearBuffer();
  // for (int16_t i = 0; i < display.width(); i += 4) {
  //   display.drawLine(0, 0, i, display.height() - 1, COLOR1);
  // }

  // Serial.println("Cleared buffer");

  // for (int16_t i = 0; i < display.height(); i += 4) {
  //   display.drawLine(display.width() - 1, 0, 0, i,
  //                    COLOR2); // on grayscale this will be mid-gray
  // }
  // display.display();
  // Serial.println("Done");
}

void loop() {
  // don't do anything!
}

void testdrawtext(const char *text, uint16_t color) {
  display.setCursor(0, 10);
  display.setTextColor(color);
  display.setTextWrap(false);
  display.setFont(&FreeSansBold12pt7b);
  display.print(text);
}

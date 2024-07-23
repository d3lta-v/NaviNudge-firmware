#include <Wire.h>
#include "Adafruit_DRV2605.h"

Adafruit_DRV2605 drv;

// See this for a basic guide to get started
// https://learn.adafruit.com/adafruit-drv2605-haptic-controller-breakout/arduino-code

// Original code documentation here
// http://adafruit.github.io/Adafruit_DRV2605_Library/html/class_adafruit___d_r_v2605.html

// See datasheet page 63 for the full list of waveforms
// https://cdn-learn.adafruit.com/assets/assets/000/113/382/original/drv2605l.pdf?1658415948


void setup() {
  Serial.begin(9600);
  Serial.println("DRV test");
  drv.begin();
    
  // I2C trigger by sending 'go' command 
  drv.setMode(DRV2605_MODE_INTTRIG); // default, internal trigger when sending GO command

  drv.selectLibrary(1);
  drv.useLRA();

  // see datasheet part 11.2
  drv.setWaveform(0, 123);
  drv.setWaveform(1, 122);
  drv.setWaveform(2, 121);
  drv.setWaveform(3, 120);
  drv.setWaveform(4, 119);
  drv.setWaveform(5, 94);
  drv.setWaveform(6, 0);  // end of waveforms
  drv.go();
  delay(4000);
  drv.setWaveform(0, 43);
  drv.setWaveform(1, 0); 
  drv.go();
  delay(4000);

  // Press EN on the board to restart the board so it does the pattern again
}

void loop() {
  
}

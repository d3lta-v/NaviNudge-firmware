#include <Arduino.h>

// This is a basic piece of code to test the integration between the IMU and the haptic motor controller

// See this for a basic guide to get started
// https://learn.adafruit.com/adafruit-drv2605-haptic-controller-breakout/arduino-code

// Original code documentation here
// http://adafruit.github.io/Adafruit_DRV2605_Library/html/class_adafruit___d_r_v2605.html

// See datasheet page 63 for the full list of waveforms
// https://cdn-learn.adafruit.com/assets/assets/000/113/382/original/drv2605l.pdf?1658415948

#include <Adafruit_BNO08x.h>
#include <Adafruit_DRV2605.h>

// #define FAST_MODE
#define BNO08X_RESET D3

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

Adafruit_DRV2605 drv;
unsigned long previousMillis = 0UL; unsigned long targetDelay = 0UL;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  if (!drv.begin()) {
    Serial.println("Could not find DRV2605");
    while (1) delay(3000);
  }
  drv.setMode(DRV2605_MODE_INTTRIG); // default, internal trigger when sending GO command
  drv.selectLibrary(2);
  drv.useLRA();

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  Serial1.begin(9600);
  // if (!bno08x.begin_I2C()) {
  if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void actuate_haptic(float desired_heading, float current_heading) {
  // 180 +- 20 = walk forward, 225+- 25 = mild right, 135+-25 = mild left
  // 270 +- 20 = sharp right, 90 +- 20 = sharp left, >290 or <70 = turn around

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= targetDelay) {
    // Finished vibration!
    previousMillis = currentMillis;
    if (current_heading - desired_heading <= 30 && current_heading - desired_heading > -30) { //walk forward 
      drv.setWaveform(0, 9);
      drv.go(); // Start playback
      targetDelay = 1500;
    }
    
    if (current_heading - desired_heading <= 70 && current_heading - desired_heading > 30) { //mild right
      drv.setWaveform(0, 39);
      drv.go();
      targetDelay = 1500;
    }

    if (current_heading - desired_heading <= 110 && current_heading - desired_heading > 70) { //sharp right
      drv.setWaveform(0, 39);
      drv.go(); 
      targetDelay = 500;
    }

    if (current_heading - desired_heading <= -110 || current_heading - desired_heading > 110) { //turn around
      drv.setWaveform(0, 53);
      drv.go(); 
      targetDelay = 500;
    }

    if (current_heading - desired_heading <= -70 && current_heading - desired_heading > -110) { //sharp left
      drv.setWaveform(0, 39);
      drv.go(); 
      targetDelay = 500;
    }

    if (current_heading - desired_heading <= -30 && current_heading - desired_heading > -70) { //mild left
      drv.setWaveform(0, 39);
      drv.go(); 
      targetDelay = 1500;
    }
  } else {
    // Vibration pattern not complete, don't actuate
  }
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    float current_heading;
    if (ypr.yaw < 0.0f) {
      current_heading = abs(ypr.yaw);
    } else {
      current_heading = 360.0 - ypr.yaw;
    }

    static long last = 0;
    long now = micros();
    Serial.print(now - last);             Serial.print("\t");
    last = now;
    Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
    Serial.print(sensorValue.un.arvrStabilizedRV.real);Serial.print("\t");
    Serial.print(sensorValue.un.arvrStabilizedRV.i);   Serial.print("\t");
    Serial.print(sensorValue.un.arvrStabilizedRV.j);   Serial.print("\t");
    Serial.print(sensorValue.un.arvrStabilizedRV.k);   Serial.print("\t");
    Serial.println(current_heading);

    actuate_haptic(180.0f, current_heading);
  }
}

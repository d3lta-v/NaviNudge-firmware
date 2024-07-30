// Compass + Haptic Motor Testing code

#include "AK09918.h"
#include "ICM20600.h"
#include <Wire.h>
#include "Adafruit_DRV2605.h"
#include "SensorFusion.h"

Adafruit_DRV2605 drv;

AK09918_err_type_t err;
int32_t x, y, z;
AK09918 ak09918;
ICM20600 icm20600(true);
int16_t acc_x, acc_y, acc_z;
int32_t offset_x, offset_y, offset_z;
float roll, pitch;
// Find the magnetic declination at your location
// http://www.magnetic-declination.com/
float declination_singapore = 0.0333f;
SF fusion;

// See this for a basic guide to get started
// https://learn.adafruit.com/adafruit-drv2605-haptic-controller-breakout/arduino-code

// Original code documentation here
// http://adafruit.github.io/Adafruit_DRV2605_Library/html/class_adafruit___d_r_v2605.html

// See datasheet page 63 for the full list of waveforms
// https://cdn-learn.adafruit.com/assets/assets/000/113/382/original/drv2605l.pdf?1658415948

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    err = ak09918.initialize();
    icm20600.initialize();
    icm20600.setPowerMode(ICM_6AXIS_LOW_NOISE);
    ak09918.switchMode(AK09918_POWER_DOWN);
    ak09918.switchMode(AK09918_CONTINUOUS_100HZ);
    Serial.begin(9600);

    err = ak09918.isDataReady();
    while (err != AK09918_ERR_OK) {
        Serial.println("Waiting Sensor");
        delay(100);
        err = ak09918.isDataReady();
    }

    drv.begin();
    drv.setMode(DRV2605_MODE_INTTRIG); // default, internal trigger when sending GO command
    drv.selectLibrary(1);
    drv.useLRA();

    Serial.println("Start figure-8 calibration after 2 seconds.");
    delay(2000);
    calibrate(10000, &offset_x, &offset_y, &offset_z);
    Serial.println("");
}

void loop() {
    // get acceleration
    acc_x = icm20600.getAccelerationX();
    acc_y = icm20600.getAccelerationY();
    acc_z = icm20600.getAccelerationZ();

    ak09918.getData(&x, &y, &z);
    x = x - offset_x;
    y = y - offset_y;
    z = z - offset_z;

    // roll/pitch in radian
    roll = atan2((float)acc_y, (float)acc_z);
    pitch = atan2(-(float)acc_x, sqrt((float)acc_y * acc_y + (float)acc_z * acc_z));
    
    double Xheading = x * cos(pitch) + y * sin(roll) * sin(pitch) + z * cos(roll) * sin(pitch);
    double Yheading = y * cos(roll) - z * sin(pitch);

    double heading = 180 + 57.3 * atan2(Yheading, Xheading) + declination_singapore;

    // Accelerometer raw data
    // Serial.print("A:  ");
    // Serial.print(acc_x);
    // Serial.print(",  ");
    // Serial.print(acc_y);
    // Serial.print(",  ");
    // Serial.print(acc_z);
    // Serial.println(" mg");

    // Gyroscope raw data
    // Serial.print("G:  ");
    // Serial.print(icm20600.getGyroscopeX());
    // Serial.print(",  ");
    // Serial.print(icm20600.getGyroscopeY());
    // Serial.print(",  ");
    // Serial.print(icm20600.getGyroscopeZ());
    // Serial.println(" dps");

    // Magnetometer raw data
    // Serial.print("M:  ");
    // Serial.print(x);
    // Serial.print(",  ");
    // Serial.print(y);
    // Serial.print(",  ");
    // Serial.print(z);
    // Serial.println(" uT");

    // Estimated roll and pitch
    // Serial.print("Roll: ");
    // Serial.println(roll * 57.3);
    // Serial.print("Pitch: ");
    // Serial.println(pitch * 57.3);

    float deltat = fusion.deltatUpdate();
    fusion.MadgwickUpdate(icm20600.getGyroscopeX(), icm20600.getGyroscopeY(), icm20600.getGyroscopeZ(), acc_x, acc_y, acc_z, x, y, z, deltat);

    // pitch = fusion.getRoll();
    // pitch = fusion.getPitch();
    // heading = fusion.getYaw();

    // Estimated heading (yaw)
    Serial.print("Heading: ");
    Serial.println(heading);
    Serial.println("--------------------------------");

    // Grab the heading value from here to test the IMU
    // For waveform types, see datasheet part 11.2
    // EDIT THIS PART OF THE CODE
    // drv.setWaveform(0, 123);
    // drv.setWaveform(1, 122);
    // drv.setWaveform(2, 121);
    // drv.setWaveform(3, 120);
    // drv.setWaveform(4, 119);
    // drv.setWaveform(5, 94);
    // drv.setWaveform(6, 0);  // end of waveforms
    // drv.go();
    // EDIT THIS PART OF THE CODE

    delay(100);
}

void calibrate(uint32_t timeout, int32_t* offsetx, int32_t* offsety, int32_t* offsetz) {
    int32_t value_x_min = 0;
    int32_t value_x_max = 0;
    int32_t value_y_min = 0;
    int32_t value_y_max = 0;
    int32_t value_z_min = 0;
    int32_t value_z_max = 0;
    uint32_t timeStart = 0;

    ak09918.getData(&x, &y, &z);

    value_x_min = x;
    value_x_max = x;
    value_y_min = y;
    value_y_max = y;
    value_z_min = z;
    value_z_max = z;
    delay(100);

    timeStart = millis();

    while ((millis() - timeStart) < timeout) {
        ak09918.getData(&x, &y, &z);

        /* Update x-Axis max/min value */
        if (value_x_min > x) {
            value_x_min = x;
            // Serial.print("Update value_x_min: ");
            // Serial.println(value_x_min);

        } else if (value_x_max < x) {
            value_x_max = x;
            // Serial.print("update value_x_max: ");
            // Serial.println(value_x_max);
        }

        /* Update y-Axis max/min value */
        if (value_y_min > y) {
            value_y_min = y;
            // Serial.print("Update value_y_min: ");
            // Serial.println(value_y_min);

        } else if (value_y_max < y) {
            value_y_max = y;
            // Serial.print("update value_y_max: ");
            // Serial.println(value_y_max);
        }

        /* Update z-Axis max/min value */
        if (value_z_min > z) {
            value_z_min = z;
            // Serial.print("Update value_z_min: ");
            // Serial.println(value_z_min);

        } else if (value_z_max < z) {
            value_z_max = z;
            // Serial.print("update value_z_max: ");
            // Serial.println(value_z_max);
        }

        Serial.print(".");
        delay(100);

    }

    *offsetx = value_x_min + (value_x_max - value_x_min) / 2;
    *offsety = value_y_min + (value_y_max - value_y_min) / 2;
    *offsetz = value_z_min + (value_z_max - value_z_min) / 2;
}
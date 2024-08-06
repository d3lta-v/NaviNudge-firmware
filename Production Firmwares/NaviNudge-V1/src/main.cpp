#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_DRV2605.h> 
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEServer.h>
#include "driver/rtc_io.h"

#define DEBUG
enum LeftRight {
  Left,
  Right
};
static const enum LeftRight handedness = Left;

// ================================ Variables =================================
int prev_state = 0; // This is the previous state of the device, so that our state machine can keep track
int master_state = 0; // This is the master state of the device. 0=Low Power, 1=Standby, 2=Walking Guidance
unsigned long time_at_initialise = 0;
unsigned long general_purpose_timer = 0;
bool IMU_ARVR_started = false;
float cache_array[32] = {0.0f}; int cache_array_index = 0; bool cache_full = false; // Caching for computation of variance
double currentBearing = 0.0;
double futureBearing = 0.0;
// ============================================================================

// ============================ Pin definitions ===============================
#define BNO08X_RESET D3 // This might change with the new PCB!!
// ============================================================================

// ============================ Sensor Classes ================================
Adafruit_BNO08x bno08x(BNO08X_RESET); sh2_SensorValue_t sensorValue;
Adafruit_DRV2605 drv;
unsigned long drvPreviousMillis = 0UL; unsigned long drvDelay = 500UL; // drvDelay describes the amount of time between each pulse
unsigned long drvCompleteMillis = 0UL; // This indicates the last time at which bearing data is available and should fire the haptic motor. Basically: when bearing not updated, DO NOT fire motor!
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;
// ============================================================================

// ==================== BLE related variables/constants =======================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
bool deviceConnected = false;
bool bluetooth_started = false;
BLECharacteristic *pCharacteristic_IMU, *pCharacteristic_Mode, *pCharacteristic_CurrentBearing, *pCharacteristic_FutureBearing;

class BTServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    time_at_initialise = millis();
#ifdef DEBUG
    Serial.println("BLE Connected!");
#endif
  };
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->startAdvertising(); // Allow reconnects
    time_at_initialise = millis(); // Transition to low power after set time
#ifdef DEBUG
    Serial.println("BLE Disconnected!");
#endif
  }
};

class BTCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    if (pCharacteristic->getUUID().equals(BLEUUID((uint16_t)0x2A5D))) {
      // Current bearing
      std::string value = pCharacteristic->getValue(); // getValue returns a string that has to be decoded
      double bearing = strtod(value.c_str(), NULL);
      if (bearing != HUGE_VAL && bearing != -HUGE_VAL) {
        // valid bearing data!!
        currentBearing = bearing;
#ifdef DEBUG
        Serial.print("Valid bearing obtained: ");
        Serial.println(bearing);
#endif
      }
#ifdef DEBUG
      else {
        Serial.println("Invalid bearings!!");
      }
#endif
      // discard bearing if invalid
      // Since we know that bearing data has been updated, it is safe to fire the haptic motors for approx. 2s
      unsigned long currentMillis = millis();
      drvCompleteMillis = currentMillis;
    } else if (pCharacteristic->getUUID().equals(BLEUUID((uint16_t)0x2A68))) {
      // Future bearing
      std::string value = pCharacteristic->getValue();
      double bearing = strtod(value.c_str(), NULL);
      if (bearing != HUGE_VAL && bearing != -HUGE_VAL) {
        // valid bearing data
        futureBearing = bearing;
      }
      // discard bearing if invalid

      // Since we know that bearing data has been updated, it is safe to fire the haptic motors for approx. 2s
      unsigned long currentMillis = millis();
      drvCompleteMillis = currentMillis;
    }
  }
};
// ============================================================================

// =========================== Helper functions ===============================
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

#ifdef DEBUG
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
#endif
}

void setReports(sh2_SensorId_t reportType, long report_interval) {
  if (!bno08x.enableReport(reportType, report_interval)) {
#ifdef DEBUG
    Serial.println("Could not enable report");
#endif
  }
}

float variance(float a[], int n) 
{
  // Compute mean (average of elements) 
  float sum = 0; 
  
  for (int i = 0; i < n; i++) sum += a[i];    
  float mean = (float)sum / (float)n; 
  // Compute sum squared differences with mean. 
  float sqDiff = 0; 
  for (int i = 0; i < n; i++) 
      sqDiff += (a[i] - mean) * (a[i] - mean); 
  return (float)sqDiff / n; 
}

void actuate_haptic(double desired_heading, double current_heading) {
  // 180 +- 20 = walk forward, 225+- 25 = mild right, 135+-25 = mild left
  // 270 +- 20 = sharp right, 90 +- 20 = sharp left, >290 or <70 = turn around

  unsigned long currentMillis = millis();
  if (currentMillis - drvPreviousMillis >= drvDelay) {
    // Finished vibration!
    drvPreviousMillis = currentMillis;
    if (current_heading - desired_heading <= 30 && current_heading - desired_heading > -30) { //walk forward 
      drv.setWaveform(0, 9);
      drv.go();
      drvDelay = 1500;
    }
    
    if (current_heading - desired_heading <= 70 && current_heading - desired_heading > 30 && handedness == Left) { //mild right
      drv.setWaveform(0, 39);
      drv.go();
      drvDelay = 1500;
    }

    if (current_heading - desired_heading <= 110 && current_heading - desired_heading > 70 && handedness == Left) { //sharp right
      drv.setWaveform(0, 39);
      drv.go();
      drvDelay = 500;
    }

    if (current_heading - desired_heading <= -110 || current_heading - desired_heading > 110) { //turn around
      drv.setWaveform(0, 53);
      drv.go();
      drvDelay = 500;
    }

    if (current_heading - desired_heading <= -70 && current_heading - desired_heading > -110 && handedness == Right) { //sharp left
      drv.setWaveform(0, 39);
      drv.go();
      drvDelay = 500;
    }

    if (current_heading - desired_heading <= -30 && current_heading - desired_heading > -70 && handedness == Right) { //mild left
      drv.setWaveform(0, 39);
      drv.go();
      drvDelay = 1500;
    }
  }
}
// ============================================================================

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
#ifdef DEBUG
  Serial.begin(115200);
#endif
  time_at_initialise = millis();

  if (!bno08x.begin_UART(&Serial1)) {
#ifdef DEBUG
    Serial.println("Failed to find BNO08x chip");
#endif
    while (1) { delay(100); }
  }
#ifdef DEBUG
  Serial.println("BNO08x Found!");
#endif
  if (!drv.begin()) {
#ifdef DEBUG
    Serial.println("Could not find DRV2605");
#endif
    while (1) delay(3000);
  }
#ifdef DEBUG
  Serial.println("DRV2605 Found!");
#endif
  drv.writeRegister8(DRV2605_REG_MODE, 0x40); // standby mode, no need to take it out of standby since it auto-resumes
}

void loop() {
  if (master_state == 0) {
    if (prev_state == 2) {
      // Place all peripherals into low power
      drv.writeRegister8(DRV2605_REG_MODE, 0x40);
      IMU_ARVR_started = false;
      prev_state = 0;
    }
    digitalWrite(LED_BUILTIN, LOW);
    if (bno08x.wasReset()) {
#ifdef DEBUG
      Serial.print("sensor was reset ");
#endif
      setReports(SH2_ACCELEROMETER, 40000); // Accelerometer only, at 25Hz/0.04s period
    }
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ACCELEROMETER) {
        // Compute variance using a rolling window to determine whether to transition to standby mode
        float cumulative_movement = sensorValue.un.accelerometer.x + sensorValue.un.accelerometer.y + sensorValue.un.accelerometer.z;
        cache_array[cache_array_index] = cumulative_movement;
        cache_array_index++;
        if (cache_array_index >= 31) {
          cache_array_index = 0;
          cache_full = true;
        }
        if (cache_full) {     // only compute variance upon full cache
          float current_variance = variance(cache_array, 32);
          if (current_variance >= 10) {
#ifdef DEBUG
            Serial.print("Accelerometer variance > 0, setting state to 1");
#endif
            prev_state = 0;
            master_state = 1; // Transition to Standby state and reset initialisation time
            time_at_initialise = millis();
          }
        }
      } 
    }

    if (millis() - time_at_initialise > 2000 && master_state == 0) {
      // Shutoff peripherals
      digitalWrite(LED_BUILTIN, HIGH);
      bno08x.hardwareReset();
      //TODO: add sleep mode to IMU
      // deep sleep for 1 seconds if system is still in low power mode
      esp_sleep_enable_timer_wakeup(1 * 1000000);
      esp_deep_sleep_start();
    }
  } 
  else if (master_state == 1) {
    if (prev_state == 2) {
      // Place all peripherals into low power
      drv.writeRegister8(DRV2605_REG_MODE, 0x40);
      IMU_ARVR_started = false;
      prev_state = 1;
    }
    digitalWrite(LED_BUILTIN, LOW);
    // Start Bluetooth
    if (!bluetooth_started) {
      if (handedness == Left) {
        BLEDevice::init("NaviNudge Node Left");
      } else {
        BLEDevice::init("NaviNudge Node Right");
      }
      // BLEDevice::setPower(ESP_PWR_LVL_N3);
      BLEServer *pServer = BLEDevice::createServer();
      pServer->setCallbacks(new BTServerCallbacks());

      // For full list of premade BLE UUIDs, refer to this https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/GATT.pdf
      BLEService *pService = pServer->createService(BLEUUID((uint16_t)0x1819)); // Create a location and navigation service
      pCharacteristic_IMU = pService->createCharacteristic(
                                          BLEUUID((uint16_t)0x2A30), // Position 3D characteristic, used for IMU
                                          // BLECharacteristic::PROPERTY_READ |
                                          // BLECharacteristic::PROPERTY_WRITE |
                                          BLECharacteristic::PROPERTY_NOTIFY
                                        );
      pCharacteristic_Mode = pService->createCharacteristic(
                                          BLEUUID((uint16_t)0x2A3F), // Alert status characteristic, used for mode switching
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
      pCharacteristic_CurrentBearing = pService->createCharacteristic(
                                          BLEUUID((uint16_t)0x2A5D), // Sensor Location characteristic, used for current bearing
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
      pCharacteristic_FutureBearing = pService->createCharacteristic(
                                          BLEUUID((uint16_t)0x2A68), // Navigation characteristic, used for future bearing
                                          BLECharacteristic::PROPERTY_READ |
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
      pCharacteristic_IMU->addDescriptor(new BLE2902());
      pCharacteristic_Mode->addDescriptor(new BLE2902());
      pCharacteristic_CurrentBearing->addDescriptor(new BLE2902());
      pCharacteristic_FutureBearing->addDescriptor(new BLE2902());
      pCharacteristic_CurrentBearing->setCallbacks(new BTCharacteristicCallbacks());
      pCharacteristic_FutureBearing->setCallbacks(new BTCharacteristicCallbacks());
      pCharacteristic_Mode->setValue(master_state); // Set to current mode

      //TODO: consider adding a steps counter characteristic for dead reckoning 
      pService->start();
      BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
      pAdvertising->addServiceUUID(pService->getUUID());
      pAdvertising->setScanResponse(true);
      // pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
      // pAdvertising->setMinPreferred(0x12);
      pAdvertising->setMinPreferred(0x0);
      pAdvertising->setMinPreferred(0x1F);

      // Set onWrite callbacks
      // BLECharacteristicCallbacks pCharacteristic_Mode_onWrite_callback = BLECharacteristicCallbacks();
      // pCharacteristic_Mode->setCallbacks

      BLEDevice::startAdvertising();
#ifdef DEBUG
      Serial.println("BLE started");
#endif
      bluetooth_started = true;
    }

    // Continuously read the current mode to detect if there was a mode switch
    unsigned long current_millis = millis();
    if (current_millis - general_purpose_timer > 2000) {
      // General purpose timer which fires every 2 seconds
      general_purpose_timer = current_millis;
      if (deviceConnected) {
        uint8_t* current_mode = pCharacteristic_Mode->getData();
        // Allow for mode transitions here! For example to walking mode
#ifdef DEBUG
        Serial.println(current_mode[0]);
#endif
        if ((int)current_mode[0] == 2) {
          // State machine transition to walking guidance mode
          prev_state = 1;
          master_state = 2;
#ifdef DEBUG
          Serial.println("Changing to walking");
#endif
        }
      }
    }

    if (millis() - time_at_initialise > (60 * 1000) && !deviceConnected) {
      // Timer for 60 seconds to transition back into low power
      prev_state = 1;
#ifdef DEBUG
      Serial.println("Transitioning back to low power");
#endif
      // Reset the accelerometer cache because the previous cache with high variance still exists!
      for (int i=0; i<32; i++) {
        cache_array[i] = 0.0;
      }
      master_state = 0;
    }
  } 
  else if (master_state == 2) {
    if (deviceConnected) {
      uint8_t* current_mode = pCharacteristic_Mode->getData();
      if ((int)current_mode[0] == 1) {
        // State machine transition to walking guidance mode
        prev_state = 2;
        master_state = 1;
#ifdef DEBUG
        Serial.println("Changing to mode 1");
#endif
      }
      if (!IMU_ARVR_started) {
        // Setup DRV as well, since it hasn't been set up yet in init
        drv.setMode(DRV2605_MODE_INTTRIG);
        drv.selectLibrary(2);
        drv.useLRA();
#ifdef DEBUG
        Serial.println("ARVR not started, resetting sensor");
#endif
        // if (bno08x.begin_UART(&Serial1)) {
          // Serial.print("sensor was reset ");
          setReports(SH2_ARVR_STABILIZED_RV, 200000); // Full IMU quaternion, 5Hz
          IMU_ARVR_started = true;
        // }
      } else {
        if (millis() - drvCompleteMillis <= 3500) {
          // actuate haptic motor only if bearing is not stale
          actuate_haptic(futureBearing, currentBearing);
        }
        if (bno08x.getSensorEvent(&sensorValue)) {
          // The "barrage" of IMU data starts here!
          digitalWrite(LED_BUILTIN, LOW);
          if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
            sh2_RotationVectorWAcc_t rotationalVector = sensorValue.un.arvrStabilizedRV;
            char buffer[64];
            snprintf(buffer, 64, "%u,%f,%f,%f,%f", sensorValue.status, rotationalVector.real, rotationalVector.i, rotationalVector.j, rotationalVector.k);
            // String str_to_send = "Hello World" + (String)current_millis;
            pCharacteristic_IMU->setValue(buffer);
            pCharacteristic_IMU->notify();
          }
        } else {
          digitalWrite(LED_BUILTIN, HIGH);
        }
      }
    } else {
      // Switch out of walking guidance due to no bluetooth!
#ifdef DEBUG
      Serial.println("Switching from mode 2 to mode 1");
#endif
      bno08x.hardwareReset();
      prev_state = 2;
      master_state = 1;
      pCharacteristic_Mode->setValue(master_state);
    }
    // String str_to_send = "Hello World" + (String)current_millis;
    // pCharacteristic_IMU->setValue(str_to_send.c_str());
    // pCharacteristic_IMU->notify();
    // Serial.println(str_to_send);
  }
}

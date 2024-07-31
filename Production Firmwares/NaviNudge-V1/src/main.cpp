#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_DRV2605.h> 
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEServer.h>
#include "driver/rtc_io.h"

// ================================ Variables =================================
int prev_state = 0; // This is the previous state of the device, so that our state machine can keep track
int master_state = 0; // This is the master state of the device. 0=Low Power, 1=Standby, 2=Walking Guidance
unsigned long time_at_initialise = 0;
unsigned long general_purpose_timer = 0;
bool IMU_ARVR_started = false;
float cache_array[32] = {0.0}; int cache_array_index = 0; bool cache_full = false; // Caching for computation of variance
// ============================================================================

// ============================ Pin definitions ===============================
#define BNO08X_RESET D3 // This might change with the new PCB!!
// ============================================================================

// ============================ Sensor Classes ================================
Adafruit_BNO08x bno08x(BNO08X_RESET); sh2_SensorValue_t sensorValue;
Adafruit_DRV2605 drv;
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
BLECharacteristic *pCharacteristic_IMU, *pCharacteristic_Mode;
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    time_at_initialise = millis();
  };
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    pServer->startAdvertising(); // Allow reconnects
    time_at_initialise = millis(); // Transition to low power after set time
  }
};
// ============================================================================

// =========================== Helper functions ===============================
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setReports(sh2_SensorId_t reportType, long report_interval) {
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable report");
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
// ============================================================================

void setup() {
  Serial.begin(115200);
  time_at_initialise = millis();

  // Do not bother with initialising peripherals EXCEPT the IMU for motion tracking
  if (!bno08x.begin_UART(&Serial1)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");
}

void loop() {
  if (master_state == 0) {
    if (bno08x.wasReset()) {
      Serial.print("sensor was reset ");
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
            master_state = 1; // Transition to Standby state and reset initialisation time
            time_at_initialise = millis();
          }
        }
      } 
    }

    if (millis() - time_at_initialise > 2000 && master_state == 0) {
      // Shutoff peripherals
      bno08x.hardwareReset();
      //TODO: add sleep mode to IMU
      // deep sleep for 1 seconds if system is still in low power mode
      esp_sleep_enable_timer_wakeup(1 * 1000000);
      rtc_gpio_isolate(GPIO_NUM_5);
      rtc_gpio_isolate(GPIO_NUM_6);
      esp_deep_sleep_start();
    }
  } 
  else if (master_state == 1) {
    if (millis() - time_at_initialise > 5 * 60 * 1000 && !deviceConnected) {
      // Timer for 5 minutes to transition back into low power
      master_state = 0;
    }

    // Start Bluetooth
    if (!bluetooth_started) {
      BLEDevice::init("NaviNudge Node");
      // BLEDevice::setPower(ESP_PWR_LVL_N3);
      BLEServer *pServer = BLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());

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
      pCharacteristic_IMU->addDescriptor(new BLE2902());
      pCharacteristic_Mode->addDescriptor(new BLE2902());
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
      BLEDevice::startAdvertising();
      Serial.println("BLE started");
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
        Serial.println(current_mode[0]);
        if ((int)current_mode[0] == 2) {
          // State machine transition to walking guidance mode
          prev_state = master_state;
          master_state = 2;
          Serial.println("Changing to walking");
        }
      }
    }
  } 
  else if (master_state == 2) {
    if (deviceConnected) {
      if (!IMU_ARVR_started) {
        Serial.println("ARVR not started, resetting sensor");
        // if (bno08x.begin_UART(&Serial1)) {
          // Serial.print("sensor was reset ");
          setReports(SH2_ARVR_STABILIZED_RV, 100000); // Full IMU quaternion, 10Hz
          IMU_ARVR_started = true;
        // }
      } else {
        if (bno08x.getSensorEvent(&sensorValue)) {
          // The "barrage" of IMU data starts here!
          if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
            sh2_RotationVectorWAcc_t rotationalVector = sensorValue.un.arvrStabilizedRV;
            char buffer[64];
            snprintf(buffer, 64, "%u,%f,%f,%f,%f", sensorValue.status, rotationalVector.real, rotationalVector.i, rotationalVector.j, rotationalVector.k);
            // String str_to_send = "Hello World" + (String)current_millis;
            pCharacteristic_IMU->setValue(buffer);
            pCharacteristic_IMU->notify();
            // Serial.print(sensorValue.status); Serial.print(",");
            // Serial.print(rotationalVector.real); Serial.print(",");
            // Serial.print(rotationalVector.i); Serial.print(",");
            // Serial.print(rotationalVector.j); Serial.print(",");
            Serial.println(rotationalVector.k); 
          }
        }
      }
    } else {
      // Switch out of walking guidance due to no bluetooth!
      Serial.println("Switching from mode 2 to mode 1");
      bno08x.hardwareReset();
      prev_state = master_state;
      master_state = 1;
      pCharacteristic_Mode->setValue(master_state);
    }
    // String str_to_send = "Hello World" + (String)current_millis;
    // pCharacteristic_IMU->setValue(str_to_send.c_str());
    // pCharacteristic_IMU->notify();
    // Serial.println(str_to_send);
  }
}
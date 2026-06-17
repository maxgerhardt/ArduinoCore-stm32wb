#include "Arduino.h"
#include "BLE.h"

void setup()
{
    BLEAddress address;
    int rssi;
    BLEScanEvent event;
    BLEAdvertisingData data;
    char name[32];
    BLEDevice device;
    
    Serial.begin(9600);
    
    while (!Serial) { }

    BLE.begin();
    BLE.setLocalName("STM32WB");

    BLE.scan();
    
    while (1) {
        if (BLE.report(address, rssi, event, data)) {
            if (data.getLocalName(name, sizeof(name))) {
                if (!strcmp(name, "STM32WB")) {

                  BLE.stopScan();
                  
                  device = BLE.connect(address);

                  break;
                }
            }
        }
    }

    // Here you could disconver the services/characteristics ...
}

void loop()
{
}

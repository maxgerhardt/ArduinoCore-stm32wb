#include "Arduino.h"
#include "BLE.h"
#include "STM32WB.h"

void setup()
{
    char string[64], name[32];
    uint8_t service_data[32], manufacturer_data[32];
    BLEAddress address;
    BLEScanEvent event;
    BLEAdvertisingData data;
    BLEUuid uuid, uuid_table[16];
    int rssi, txPower, index, count, length;
    uint16_t interval_min, interval_max, major, minor;
    
    Serial.begin(9600);
    
    while (!Serial) { }

    BLE.begin();
    BLE.setLocalName("STM32WB");

    Serial.println();
    Serial.println();

    BLE.address().toString(string, sizeof(string) -1);    

    Serial.println(string);
    Serial.println();

    BLE.scan(BLE_SCAN_MODE_WITHOUT_DUPLICATES);

    while (1) {
        if (BLE.report(address, rssi, event, data)) {
            address.toString(string, sizeof(string) -1);

            Serial.print(string);
            Serial.print(" (");
            Serial.print(address.type());
            Serial.print("), ");
            Serial.print(rssi);
            
            if (event & BLE_SCAN_EVENT_CONNECTABLE) {
                Serial.print(", connectable");
            }

            if (event & BLE_SCAN_EVENT_ADVERTISEMENT) {
                Serial.print(", advertisement");
            }

            if (event & BLE_SCAN_EVENT_RESPONSE) {
                Serial.print(", response");
            }

            if (data.getTxPowerLevel(txPower)) {
                Serial.print(", tx_power_level = ");
                Serial.print(txPower);
            }

            if (data.getConnectionInterval(interval_min, interval_max)) {
                Serial.print(", connection_interval = ");
                Serial.print(interval_min);
                Serial.print(" / ");
                Serial.print(interval_max);
            }

            if (data.getLocalName(name, sizeof(name))) {
                Serial.print(", name = \"");
                Serial.print(name);
                Serial.print("\"");
            }

            if (data.getServiceUuids(uuid_table, (sizeof(uuid_table) / sizeof(uuid_table[0])), count)) {
                if (count > 1) {
                    Serial.print(", service_uuids = {");
                } else {
                    Serial.print(", service_uuid =");
                }

                for (index = 0; index < count; index++) {
                    uuid_table[index].toString(string, sizeof(string) -1);

                    if (index) {
                        Serial.print(", ");
                    } else {
                        Serial.print(" ");
                    }

                    Serial.print(string);
                }

                if (count > 1) {
                    Serial.print(" }");
                }
            }
            
            if (data.getServiceData(uuid, service_data, sizeof(service_data), length)) {
                uuid.toString(string, sizeof(string) -1);

                Serial.print(", service_data = ");
                Serial.print(string);

                if (length) {
                    Serial.print(" / ");

                    for (index = 0; index < length; index++) {
                        Serial.print(service_data[index] >> 4, HEX);
                        Serial.print(service_data[index] & 15, HEX);
                    }
                }
            }
            
            if (data.getManufacturerData(manufacturer_data, sizeof(manufacturer_data), length)) {
                Serial.print(", manufacturer_data = ");
                Serial.print(manufacturer_data[1] >> 4, HEX);
                Serial.print(manufacturer_data[1] & 15, HEX);
                Serial.print(manufacturer_data[0] >> 4, HEX);
                Serial.print(manufacturer_data[0] & 15, HEX);

                if (length >= 2) {
                    Serial.print(" / ");

                    for (index = 2; index < length; index++) {
                        Serial.print(manufacturer_data[index] >> 4, HEX);
                        Serial.print(manufacturer_data[index] & 15, HEX);
                    }
                }
            }

            if (data.getBeacon(uuid, major, minor, rssi)) {
                uuid.toString(string, sizeof(string) -1);

                Serial.print(", beacon = \"");
                Serial.print(string);
                Serial.print("\" / ");
                Serial.print(major);
                Serial.print(" / ");
                Serial.print(minor);
                Serial.print(" / ");
                Serial.print(rssi);
            }
            
            Serial.println();
        }
    }
}

void loop()
{
}

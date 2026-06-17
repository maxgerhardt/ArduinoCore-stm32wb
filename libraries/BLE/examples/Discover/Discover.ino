/* Simpe peripheral that discovers the central's GATT database. Connect via a cellphone / "nRF Connect".
 */
#include "Arduino.h"
#include "BLE.h"

void setup()
{
    Serial.begin(9600);
    
    while (!Serial) { }

    BLE.begin();
    BLE.setLocalName("STM32WB");
}

void loop()
{
    BLEDevice device;
    BLEService service;
    BLECharacteristic characteristic;
    BLEUuid uuid;
    char string[64];

    if (!BLE.advertising() && !BLE.connected()) {
        BLE.advertise();
    }

    if (BLE.connected()) {
        static bool discovered = false;

        if (!discovered) {
            device = BLE.central();

            device.discoverServices();

            for (unsigned int index = 0; index < device.serviceCount(); index++) {
                service = device.service(index);

                uuid = service.uuid();

                uuid.toString(string, sizeof(string) -1);

                Serial.print("SERVICE = ");
                Serial.println(string);

                device.discoverCharacteristics(uuid);
                
                for (unsigned int index = 0; index < service.characteristicCount(); index++) {
                    characteristic = service.characteristic(index);
                
                    uuid = characteristic.uuid();
                
                    uuid.toString(string, sizeof(string) -1);
                
                    Serial.print("  CHARACTERISTIC = ");
                    Serial.println(string);
                }
            }

            discovered = true;
        }
    }
}

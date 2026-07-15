/*
 * Copyright (c) 2020-2026 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#ifndef BLE_H
#define BLE_H

#include <Arduino.h>
#include <type_traits>

#include "stm32wb_fwu.h"

enum BLEOption : uint32_t {
    BLE_OPTION_ROLE_PERIPHERAL                               = 0x00000001,
    BLE_OPTION_ROLE_BROADCASTER                              = 0x00000002,
    BLE_OPTION_ROLE_CENTRAL                                  = 0x00000004,
    BLE_OPTION_ROLE_OBSERVER                                 = 0x00000008,
    BLE_OPTION_RANDOM_STATIC_ADDRESS                         = 0x00000010,
    BLE_OPTION_PRIVACY                                       = 0x00000020,
};

enum BLEAddressType : uint8_t {
    BLE_ADDRESS_TYPE_PUBLIC                                  = 0,
    BLE_ADDRESS_TYPE_RANDOM_STATIC                           = 1,
    BLE_ADDRESS_TYPE_RANDOM_PRIVATE_RESOLVABLE               = 2,
    BLE_ADDRESS_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE           = 3,
    BLE_ADDRESS_TYPE_NONE                                    = 255
};

enum BLEUuidType : uint8_t {
    BLE_UUID_TYPE_16                                         = 0,
    BLE_UUID_TYPE_128                                        = 1,
    BLE_UUID_TYPE_NONE                                       = 255
};

enum BLEAppearance : uint16_t {
    BLE_APPEARANCE_UNKNOWN                                   = 0,
    BLE_APPEARANCE_GENERIC_PHONE                             = 64,
    BLE_APPEARANCE_GENERIC_COMPUTER                          = 128,
    BLE_APPEARANCE_GENERIC_WATCH                             = 192,
    BLE_APPEARANCE_WATCH_SPORTS_WATCH                        = 193,
    BLE_APPEARANCE_GENERIC_CLOCK                             = 256,
    BLE_APPEARANCE_GENERIC_DISPLAY                           = 320,
    BLE_APPEARANCE_GENERIC_REMOTE_CONTROL                    = 384,
    BLE_APPEARANCE_GENERIC_EYE_GLASSES                       = 448,
    BLE_APPEARANCE_GENERIC_TAG                               = 512,
    BLE_APPEARANCE_GENERIC_KEYRING                           = 576,
    BLE_APPEARANCE_GENERIC_MEDIA_PLAYER                      = 640,
    BLE_APPEARANCE_GENERIC_BARCODE_SCANNER                   = 704,
    BLE_APPEARANCE_GENERIC_THERMOMETER                       = 768,
    BLE_APPEARANCE_THERMOMETER_EAR                           = 769,
    BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR                 = 832,
    BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT         = 833,
    BLE_APPEARANCE_GENERIC_BLOOD_PRESSURE                    = 896,
    BLE_APPEARANCE_BLOOD_PRESSURE_ARM                        = 897,
    BLE_APPEARANCE_BLOOD_PRESSURE_WRIST                      = 898,
    BLE_APPEARANCE_GENERIC_HID                               = 960,
    BLE_APPEARANCE_HID_KEYBOARD                              = 961,
    BLE_APPEARANCE_HID_MOUSE                                 = 962,
    BLE_APPEARANCE_HID_JOYSTICK                              = 963,
    BLE_APPEARANCE_HID_GAMEPAD                               = 964,
    BLE_APPEARANCE_HID_DIGITIZERSUBTYPE                      = 965,
    BLE_APPEARANCE_HID_CARD_READER                           = 966,
    BLE_APPEARANCE_HID_DIGITAL_PEN                           = 967,
    BLE_APPEARANCE_HID_BARCODE                               = 968,
    BLE_APPEARANCE_GENERIC_GLUCOSE_METER                     = 1024,
    BLE_APPEARANCE_GENERIC_RUNNING_WALKING_SENSOR            = 1088,
    BLE_APPEARANCE_RUNNING_WALKING_SENSOR_IN_SHOE            = 1089,
    BLE_APPEARANCE_RUNNING_WALKING_SENSOR_ON_SHOE            = 1090,
    BLE_APPEARANCE_RUNNING_WALKING_SENSOR_ON_HIP             = 1091,
    BLE_APPEARANCE_GENERIC_CYCLING                           = 1152,
    BLE_APPEARANCE_CYCLING_CYCLING_COMPUTER                  = 1153,
    BLE_APPEARANCE_CYCLING_SPEED_SENSOR                      = 1154,
    BLE_APPEARANCE_CYCLING_CADENCE_SENSOR                    = 1155,
    BLE_APPEARANCE_CYCLING_POWER_SENSOR                      = 1156,
    BLE_APPEARANCE_CYCLING_SPEED_CADENCE_SENSOR              = 1157,
    BLE_APPEARANCE_GENERIC_PULSE_OXIMETER                    = 3136,
    BLE_APPEARANCE_PULSE_OXIMETER_FINGERTIP                  = 3137,
    BLE_APPEARANCE_PULSE_OXIMETER_WRIST_WORN                 = 3138,
    BLE_APPEARANCE_GENERIC_WEIGHT_SCALE                      = 3200,
    BLE_APPEARANCE_GENERIC_OUTDOOR_SPORTS_ACT                = 5184,
    BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_DISP               = 5185,
    BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_AND_NAV_DISP       = 5186,
    BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_POD                = 5187,
    BLE_APPEARANCE_OUTDOOR_SPORTS_ACT_LOC_AND_NAV_POD        = 5188,
};

enum BLEAdType : uint8_t {
    BLE_AD_TYPE_FLAGS                                        = 0x01,
    BLE_AD_TYPE_16_BIT_SERV_UUID                             = 0x02,
    BLE_AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST                  = 0x03,
    BLE_AD_TYPE_128_BIT_SERV_UUID                            = 0x06,
    BLE_AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST                 = 0x07,
    BLE_AD_TYPE_SHORTENED_LOCAL_NAME                         = 0x08,
    BLE_AD_TYPE_COMPLETE_LOCAL_NAME                          = 0x09,
    BLE_AD_TYPE_TX_POWER_LEVEL                               = 0x0a,
    BLE_AD_TYPE_PERIPHERAL_CONN_INTERVAL                     = 0x12,
    BLE_AD_TYPE_SERV_SOLICIT_16_BIT_UUID_LIST                = 0x14,
    BLE_AD_TYPE_SERV_SOLICIT_128_BIT_UUID_LIST               = 0x15,
    BLE_AD_TYPE_SERVICE_DATA_16_BIT_UUID                     = 0x16,
    BLE_AD_TYPE_APPEARANCE                                   = 0x19,
    BLE_AD_TYPE_SERVICE_DATA_128_BIT_UUID                    = 0x21,
    BLE_AD_TYPE_MANUFACTURER_SPECIFIC_DATA                   = 0xff
};

enum BLEAdFlags : uint8_t {
    BLE_AD_FLAGS_LE_LIMITED_DISCOVERABLE_MODE                = 0x01,
    BLE_AD_FLAGS_LE_GENERAL_DISCOVERABLE_MODE                = 0x02,
    BLE_AD_FLAGS_BR_EDR_NOT_SUPPORTED                        = 0x04,
    BLE_AD_FLAGS_LE_BR_EDR_CONTROLLER                        = 0x08,
    BLE_AD_FLAGS_LE_BR_EDR_HOST                              = 0x10
};

enum BLEProperty : uint8_t {
    BLE_PROPERTY_READ                                        = 0x02,
    BLE_PROPERTY_WRITE_WITHOUT_RESPONSE                      = 0x04,
    BLE_PROPERTY_WRITE                                       = 0x08,
    BLE_PROPERTY_NOTIFY                                      = 0x10,
    BLE_PROPERTY_INDICATE                                    = 0x20
};

enum BLEPermission : uint8_t {
    BLE_PERMISSION_READ_UNENCRYPTED                          = 0x00,
    BLE_PERMISSION_READ_UNAUTHENTICATED                      = 0x01,
    BLE_PERMISSION_READ_AUTHENTICATED                        = 0x02,
    BLE_PERMISSION_READ_AUTHENTICATED_SECURE_CONNECTION      = 0x03,
    BLE_PERMISSION_WRITE_UNENCRYPTED                         = 0x00,
    BLE_PERMISSION_WRITE_UNAUTHENTICATED                     = 0x04,
    BLE_PERMISSION_WRITE_AUTHENTICATED                       = 0x08,
    BLE_PERMISSION_WRITE_AUTHENTICATED_SECURE_CONNECTION     = 0x0c,
    BLE_PERMISSION_SUBSCRIBE_UNENCRYPTED                     = 0x00,
    BLE_PERMISSION_SUBSCRIBE_UNAUTHENTICATED                 = 0x10,
    BLE_PERMISSION_SUBSCRIBE_AUTHENTICATED                   = 0x20,
    BLE_PERMISSION_SUBSCRIBE_AUTHENTICATED_SECURE_CONNECTION = 0x30
};

enum BLESecurity : uint8_t {
    BLE_SECURITY_UNENCRYPTED                                 = 0x00,
    BLE_SECURITY_UNAUTHENTICATED                             = 0x01,
    BLE_SECURITY_AUTHENTICATED                               = 0x02,
    BLE_SECURITY_AUTHENTICATED_SECURE_CONNECTION             = 0x03,
};

enum BLEPhy : uint8_t {
    BLE_PHY_1M                                               = 0x01,
    BLE_PHY_2M                                               = 0x02,
    BLE_PHY_CODED                                            = 0x04
};

enum BLEPhyOption : uint16_t {
    BLE_PHY_OPTION_S2                                        = 0x01,
    BLE_PHY_OPTION_S8                                        = 0x02,
};

enum BLEDiscoverable : uint8_t {
    BLE_DISCOVERABLE_LIMITED                                 = 0x01,
    BLE_DISCOVERABLE_GENERAL                                 = 0x02,
};

enum BLEScanType : uint8_t {
    BLE_SCAN_TYPE_PASSIVE                                    = 0x00,
    BLE_SCAN_TYPE_ACTIVE                                     = 0x01,
};

enum BLEScanMode : uint8_t {
    BLE_SCAN_MODE_NONE                                       = 0x00,
    BLE_SCAN_MODE_WITHOUT_DUPLICATES                         = 0x01,
    BLE_SCAN_MODE_CONNECTABLE                                = 0x02,
    BLE_SCAN_MODE_FILTER_LIST                                = 0x04,
};

enum BLEScanEvent : uint8_t {
    BLE_SCAN_EVENT_NONE                                      = 0x00,
    BLE_SCAN_EVENT_ADVERTISEMENT                             = 0x01,
    BLE_SCAN_EVENT_RESPONSE                                  = 0x02,
    BLE_SCAN_EVENT_CONNECTABLE                               = 0x04,
};

enum BLEWriteType : uint8_t {
    BLE_WRITE_TYPE_DEFAULT                                   = 0x00,
    BLE_WRITE_TYPE_WITH_RESPONSE                             = 0x01,
    BLE_WRITE_TYPE_WITHOUT_RESPONSE                          = 0x02,
};

enum BLEStatus : uint8_t {
    BLE_STATUS_SUCCESS                                       = 0,
    BLE_STATUS_FAILURE                                       = 1,
    BLE_STATUS_BUSY                                          = 255
};
  
enum BLEPayload : uint8_t {
    BLE_PAYLOAD_PRBS9                                        = 0,
    BLE_PAYLOAD_PATTERN_11110000                             = 1,
    BLE_PAYLOAD_PATTERN_10101010                             = 2,
    BLE_PAYLOAD_PRBS15                                       = 3,
    BLE_PAYLOAD_PATTERN_11111111                             = 4,
    BLE_PAYLOAD_PATTERN_00000000                             = 5,
    BLE_PAYLOAD_PATTERN_00001111                             = 6,
    BLE_PAYLOAD_PATTERN_01010101                             = 7,
};

#define BLE_ADDRESS_SIZE             6
#define BLE_UUID_SIZE                16
#define BLE_AD_SIZE                  31
#define BLE_MAX_AD_LENGTH            29
#define BLE_MIN_ATT_MTU              23
#define BLE_MAX_ATT_MTU              247
#define BLE_MIN_ATT_SIZE             20
#define BLE_MAX_ATT_SIZE             244
#define BLE_MIN_DEVICE_NAME_LENGTH   0
#define BLE_MAX_DEVICE_NAME_LENGTH   244

#define BLE_PASSKEY_UNDEFINED        0xffffffff

class BLEAddress {
public:
    BLEAddress();
    BLEAddress(const uint8_t data[BLE_ADDRESS_SIZE], BLEAddressType type);
    BLEAddress(const char *string, BLEAddressType type);

    operator bool() const;
    bool operator==(const BLEAddress &other) const;
    bool operator!=(const BLEAddress &other) const;
  
    BLEAddressType type() const;
    const uint8_t *data() const;
    int toString(char* string, int size, bool stripped = false) const;
    String toString(bool stripped = false) const;
    
private:
    BLEAddressType m_type;
    uint8_t m_data[BLE_ADDRESS_SIZE];
};

class BLEUuid {
public:
    BLEUuid();
    BLEUuid(uint16_t);
    BLEUuid(const uint8_t data[BLE_UUID_SIZE]);
    BLEUuid(const uint8_t data[], BLEUuidType type);
    BLEUuid(const char *string);

    operator bool() const;
    bool operator==(const BLEUuid &other) const;
    bool operator!=(const BLEUuid &other) const;
  
    BLEUuidType type() const;
    const uint8_t *data() const;


    int toString(char* string, int size, bool stripped = false) const;
    String toString(bool stripped = false) const;
    
private:
    BLEUuidType m_type;
    uint8_t m_data[BLE_UUID_SIZE];
};

class BLEAdvertisingData {
public:
    BLEAdvertisingData();
    BLEAdvertisingData(BLEAdFlags flags);
    BLEAdvertisingData(const uint8_t data[], int length);

    operator bool() const;

    int length() const;
    const uint8_t *data() const;

    bool setFlags(BLEAdFlags flags);
    bool setTxPowerLevel(int tx_power_level);
    bool setConnectionInterval(uint16_t interval_min, uint16_t interval_max);
    bool setLocalName(const char *local_name);
    bool setAppearance(BLEAppearance apperance);
    bool setServiceUuid(const char *uuid);
    bool setServiceUuid(const BLEUuid &uuid);
    bool setServiceSolicitation(const char *uuid);
    bool setServiceSolicitation(const BLEUuid &uuid);
    bool setServiceData(const char *uuid, const uint8_t data[], int length);
    bool setServiceData(const BLEUuid &uuid, const uint8_t data[], int length);
    bool setManufacturerData(const uint8_t data[], int length);
    bool setBeacon(const char *uuid, uint16_t major, uint16_t minor, int rssi);
    bool setBeacon(const BLEUuid &uuid, uint16_t major, uint16_t minor, int rssi);

    bool getFlags(BLEAdFlags &flags);
    bool getTxPowerLevel(int &tx_power_level);
    bool getConnectionInterval(uint16_t &interval_min, uint16_t &interval_max);
    bool getAppearance(BLEAppearance &apperance);
    bool getLocalName(char *local_name, int size);
    bool getServiceUuid(const char *uuid);
    bool getServiceUuid(const BLEUuid &uuid);
    bool getServiceUuids(BLEUuid uuid[], int size, int &count);
    bool getServiceSolicitation(BLEUuid uuid[], int size, int &count);
    bool getServiceData(BLEUuid &uuid, uint8_t data[], int size, int &length);
    bool getManufacturerData(uint8_t data[], int size, int &length);
    bool getBeacon(BLEUuid &uuid, uint16_t &major, uint16_t &minor, int &rssi);
  
private:
    uint8_t m_length;
    uint8_t m_data[BLE_AD_SIZE];

    void clear();
    int  contains(BLEAdType type) const;
    bool add(BLEAdType type, const uint8_t data[], int length);
    bool remove(BLEAdType type);
    int  get(BLEAdType type, uint8_t data[], int size);

    friend class BLELocalDevice;
};

class BLECharacteristic {
public:
    BLECharacteristic();
    BLECharacteristic(class BLECharacteristicInterface *interface);
    BLECharacteristic(const char *uuid, uint8_t properties, uint8_t permissions, const void *value, int valueLength, int valueSize, bool fixedLength);
    BLECharacteristic(const BLEUuid &uuid, uint8_t properties, uint8_t permissions, const void *value, int valueLength, int valueSize, bool fixedLength);
    BLECharacteristic(const char *uuid, uint8_t properties, uint8_t permissions, int valueSize, bool fixedLength);
    BLECharacteristic(const BLEUuid &uuid, uint8_t properties, uint8_t permissions, int valueSize, bool fixedLength);
    BLECharacteristic(const char* uuid, uint8_t properties, uint8_t permissions, const char* value);
    BLECharacteristic(const BLEUuid &uuid, uint8_t properties, uint8_t permissions, const char* value);
    BLECharacteristic(const char* uuid, uint8_t properties, const char* value);
    BLECharacteristic(const BLEUuid &uuid, uint8_t properties, const char* value);
    ~BLECharacteristic();

    BLECharacteristic(const BLECharacteristic &other);
    BLECharacteristic &operator=(const BLECharacteristic &other);
    BLECharacteristic(BLECharacteristic &&other);
    BLECharacteristic &operator=(BLECharacteristic &&other);
    
    operator bool() const;

    const BLEUuid uuid() const;
    uint8_t properties() const;
    bool fixedLength() const;

    int valueSize();
    int getValue(void *value, int size);

    bool setValue(const void *value, int length);

    bool notifyValue(const void *value, int length);
    bool notifyValue(const void *value, int length, void(*callback)(void));
    bool notifyValue(const void *value, int length, Callback callback);

    int readValue(void *value, int size);

    bool writeValue(const void *value, int length, BLEWriteType writeType = BLE_WRITE_TYPE_DEFAULT);
    bool writeValue(const void *value, int length, BLEWriteType writeType, void(*callback)(void));
    bool writeValue(const void *value, int length, BLEWriteType writeType, Callback callback);

    bool read();
    bool read(void(*callback)(void));
    bool read(Callback callback);
    bool subscribe();
    bool unsubscribe();

    uint8_t status();
    bool busy();
    bool written();
    bool subscribed();
    
    void onNotify(void(*callback)(void));
    void onNotify(Callback callback);
    void onRead(void(*callback)(void));
    void onRead(Callback callback);
    void onWrite(void(*callback)(void));
    void onWrite(Callback callback);
    void onSubscribe(void(*callback)(void));
    void onSubscribe(Callback callback);

    inline class BLECharacteristicInterface *interface() const { return m_interface; };

public:
    template<typename T>
    BLECharacteristic(const char *uuid, uint8_t properties, const T& value) :
        BLECharacteristic(uuid, properties, 0, reinterpret_cast<const void*>(&value), sizeof(T), sizeof(T), false) { };

    template<typename T>
    BLECharacteristic(const BLEUuid &uuid, uint8_t properties, const T& value) :
        BLECharacteristic(uuid, properties, 0, reinterpret_cast<const void*>(&value), sizeof(T), sizeof(T), false) { };
  
    template<typename T>
    BLECharacteristic(const char *uuid, uint8_t properties, uint8_t permissions, const T& value) :
        BLECharacteristic(uuid, properties, permissions, reinterpret_cast<const void*>(&value), sizeof(T), sizeof(T), false) { };

    template<typename T>
    BLECharacteristic(const BLEUuid &uuid, uint8_t properties, uint8_t permissions, const T& value) :
        BLECharacteristic(uuid, properties, permissions, reinterpret_cast<const void*>(&value), sizeof(T), sizeof(T), false) { };
  
    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, int>::type
    getValue(T& value) {
        return getValue(reinterpret_cast<void*>(&value), sizeof(T));
    }
    
    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, bool>::type
    setValue(const T& value) {
        return setValue(reinterpret_cast<const void*>(&value), sizeof(T));
    }
    
    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, bool>::type
    notifyValue(const T& value) {
        return notifyValue(reinterpret_cast<const void*>(&value), sizeof(T));
    }

    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, bool>::type
    notifyValue(const T& value, void(*callback)(void)) {
        return notifyValue(reinterpret_cast<const void*>(&value), sizeof(T), callback);
    }

    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, bool>::type
    notifyValue(const T& value, Callback callback) {
        return notifyValue(reinterpret_cast<const void*>(&value), sizeof(T), callback);
    }

    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, int>::type
    readValue(T& value) {
        return readValue(reinterpret_cast<void*>(&value), sizeof(T));
    }
  
    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, bool>::type
    writeValue(const T& value, BLEWriteType writeType) {
        return writeValue(reinterpret_cast<const void*>(&value), sizeof(T), writeType);
    }
  
    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, bool>::type
    writeValue(const T& value, BLEWriteType writeType, void(*callback)(void)) {
        return writeValue(reinterpret_cast<const void*>(&value), sizeof(T), writeType, callback);
    }

    template<typename T>
    typename std::enable_if<std::is_standard_layout<T>::value, bool>::type
    writeValue(const T& value, BLEWriteType writeType, Callback callback) {
        return writeValue(reinterpret_cast<const void*>(&value), sizeof(T), writeType, callback);
    }
  
private:
    static BLECharacteristicInterface *construct(const BLEUuid &uuid, uint8_t properties, uint8_t permissions, const void *value, int valueLength, int valueSize, bool fixedLength, bool constant);

    BLECharacteristicInterface *m_interface;
};


class BLEService {
public:
    BLEService();
    BLEService(class BLEServiceInterface *interface);
    BLEService(const char *uuid);
    BLEService(const BLEUuid &uuid);
    ~BLEService();

    BLEService(const BLEService &other);
    BLEService &operator=(const BLEService &other);
    BLEService(BLEService &&other);
    BLEService &operator=(BLEService &&other);
    
    operator bool() const;

    const BLEUuid uuid() const;

    unsigned int characteristicCount();
    BLECharacteristic characteristic(unsigned int index);
    BLECharacteristic characteristic(const char *uuid);
    BLECharacteristic characteristic(const BLEUuid &uuid);
    bool addCharacteristic(BLECharacteristic &characteristic);

    inline class BLEServiceInterface *interface() const { return m_interface; };
  
private:
    static BLEServiceInterface *construct(const BLEUuid &uuid);

    BLEServiceInterface *m_interface;
};


class BLEDevice {
public:
    BLEDevice();
    BLEDevice(class BLEDeviceInterface *interface);
    ~BLEDevice();

    BLEDevice(const BLEDevice &other);
    BLEDevice &operator=(const BLEDevice &other);
    BLEDevice(BLEDevice &&other);
    BLEDevice &operator=(BLEDevice &&other);
    
    operator bool() const;

    const BLEAddress address() const;
    int mtu();

    void disconnect();
    bool connected();

    bool discover();
    bool discover(void(*callback)(void));
    bool discover(Callback callback);
    bool discoverServices();
    bool discoverServices(void(*callback)(void));
    bool discoverServices(Callback callback);
    bool discoverCharacteristics(const char *uuid);
    bool discoverCharacteristics(const char *uuid, void(*callback)(void));
    bool discoverCharacteristics(const char *uuid, Callback callback);
    bool discoverCharacteristics(const BLEUuid &uuid);
    bool discoverCharacteristics(const BLEUuid &uuid, void(*callback)(void));
    bool discoverCharacteristics(const BLEUuid &uuid, Callback callback);
    bool discoverCharacteristics(BLEService service);
    bool discoverCharacteristics(BLEService service, void(*callback)(void));
    bool discoverCharacteristics(BLEService service, Callback callback);
    unsigned int serviceCount();
    BLEService service(unsigned int index);
    BLEService service(const char *uuid);
    BLEService service(const BLEUuid &uuid);

    void getPhys(BLEPhy &tx_phy, BLEPhy &rx_phy);
    bool setConnectionInterval(uint16_t interval_min, uint16_t interval_max);
    bool setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency);
    bool setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout);
    void getConnectionParameters(uint16_t &interval, uint16_t &latency, uint16_t &timeout); 
    
    void onMTUExchange(void(*callback)(void));
    void onMTUExchange(Callback callback);
    void onConnectionParameters(void(*callback)(void));
    void onConnectionParameters(Callback callback);
    void onDisconnect(void(*callback)(void));
    void onDisconnect(Callback callback);
  
    inline class BLEDeviceInterface *interface() const { return m_interface; };
  
protected:

private:
    BLEDeviceInterface *m_interface;
};


class BLEClass {
public:
    static bool begin();
    static bool begin(uint32_t options);
    static bool begin(uint32_t options, int mtu);
    static void end();
    
    static const BLEAddress address();
 
    // HCI / L2CAP
    static bool setTxPowerLevel(int txPower);
    static bool setPreferredPhy(BLEPhy txPhy, BLEPhy rxPhy, BLEPhyOption phyOptions = (BLEPhyOption)0);
    static bool setConnectionInterval(uint16_t interval_min, uint16_t interval_max);
    static bool setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency);
    static bool setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout);

    // GAP Peripheral/Central
    static bool setDeviceName(const char *deviceName);
    static bool setAppearance(BLEAppearance appearance);

    // GAP Peripheral
    static bool setSecurity(BLESecurity security);
    static bool setPasskey(uint32_t passkey);
    static bool setBonding(bool bonding);
    static void setConnectable(bool connectable);
    static void setDiscoverable(BLEDiscoverable discoverable);
    static bool setAdvertisingInterval(uint16_t advertisingInterval);
    static bool setAdvertisingInterval(uint16_t minimumAdvertisingInterval, uint16_t maximumAdvertisingInterval);
    static bool setAdvertisingData(const BLEAdvertisingData &advertising_data);
    static void setAdvertisingData();
    static bool setScanResponseData(const BLEAdvertisingData &advertising_data);
    static void setScanResponseData();
    static bool setLocalName(const char *localName);
    static bool setServiceUuid(const char *uuid);
    static bool setServiceUuid(const BLEUuid &uuid);
    static bool setServiceSolicitation(const char *uuid);
    static bool setServiceSolicitation(const BLEUuid &uuid);
    static bool setServiceData(const char *uuid, const uint8_t data[], int length);
    static bool setServiceData(const BLEUuid &uuid, const uint8_t data[], int length);
    static bool setManufacturerData(const uint8_t data[], int length);
    static bool setBeacon(const char *uuid, uint16_t major, uint16_t minor, int rssi);
    static bool setBeacon(const BLEUuid &uuid, uint16_t major, uint16_t minor, int rssi);
    static bool advertise();
    static void stopAdvertise();
    static bool advertising();
    static BLEDevice central();
    static bool connected();
    static void disconnect();

    // GAP Central
    static bool setScanInterval(uint16_t scanInterval);
    static bool setScanWindow(uint16_t scanWindow);
    static void setScanType(BLEScanType scanType);
    static bool setFilterList(const BLEAddress address[], int count);
    static bool scan(uint8_t scanMode = BLE_SCAN_MODE_WITHOUT_DUPLICATES);
    static void stopScan();
    static bool scanning();
    static unsigned int reportCount();
    static bool report(BLEAddress &address, int &rssi, BLEScanEvent &event, BLEAdvertisingData &data);
    static BLEDevice connect(BLEAddress address, uint32_t passkey = BLE_PASSKEY_UNDEFINED);
    static BLEDevice connect(BLEAddress address, uint32_t passkey, void(*callback)(void));
    static BLEDevice connect(BLEAddress address, uint32_t passkey, Callback callback);

    // GATT Server
    static bool activate();
    static unsigned int serviceCount();
    static BLEService service(unsigned int index);
    static BLEService service(const char *uuid);
    static BLEService service(const BLEUuid &uuid);
    static bool addService(BLEService &service);

    // Bond Storage
    static int  queryBondStorage(BLEAddress address[], int count);
    static bool removeBondStorage(const BLEAddress &address);
    static bool clearBondStorage();
  
    //Test Mode
    static bool testTransmit(int channel, int length, BLEPayload payload, BLEPhy phy = BLE_PHY_1M);
    static bool testReceive(int channel, BLEPhy phy = BLE_PHY_1M, int modulation = 0);
    static uint32_t testStop();
    static bool testing();

    static void onStop(void(*callback)(void));
    static void onStop(Callback callback);
    static void onReport(void(*callback)(void));
    static void onReport(Callback callback);
    static void onConnect(void(*callback)(void));
    static void onConnect(Callback callback);
    static void onDisconnect(void(*callback)(void));
    static void onDisconnect(Callback callback);
};

extern BLEClass BLE;


enum BLEUartProtocol : uint8_t {
    BLE_UART_PROTOCOL_BLUEST = 0,
    BLE_UART_PROTOCOL_NORDIC,
    BLE_UART_PROTOCOL_HMSOFT
};

#define BLE_UART_RX_BUFFER_SIZE 128
#define BLE_UART_TX_BUFFER_SIZE 128

class BLEUart : public Stream, public BLEService {
public:
    BLEUart(BLEUartProtocol protocol);
    ~BLEUart();
    
    operator bool();

    int available() override;
    int peek() override;
    int read() override;
    int read(uint8_t *buffer, size_t size) override;

    int availableForWrite() override;
    size_t write(uint8_t data) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    void flush() override;

    void setNonBlocking(bool enable);

    void onReceive(void(*callback)(void));
    void onReceive(Callback callback);
  
    using Print::write;
    
private:
    BLECharacteristic m_rx_characteristic;
    BLECharacteristic m_tx_characteristic;
    BLECharacteristic m_tx2_characteristic;

    uint8_t m_rx_data[BLE_UART_RX_BUFFER_SIZE];
    uint8_t m_tx_data[BLE_UART_TX_BUFFER_SIZE];
    volatile uint16_t m_rx_write;
    volatile uint16_t m_rx_read;
    volatile uint16_t m_rx_count;
    volatile uint16_t m_tx_write;
    volatile uint16_t m_tx_read;
    volatile uint16_t m_tx_count;
    volatile uint16_t m_tx_size;
    volatile uint8_t m_tx_busy;
    volatile uint8_t m_connected;

    const BLEUartProtocol m_protocol;
    uint8_t m_packet_size;
    uint8_t m_nonblocking;

    Callback m_receive_callback;

    void transmit();
    void receive();
    void connect();
};

#define BLE_OTA_QUEUE_SIZE       4096

enum BLEOtaOption : uint32_t {
    BLE_OTA_OPTION_TRIAL                                     = 0x00000001,
    BLE_OTA_OPTION_CANDIDATE                                 = 0x00000002,
    BLE_OTA_OPTION_WIRELESS                                  = 0x00000004,
};

enum BLEOtaState : uint8_t {
    BLE_OTA_STATE_READY                                      = 0x00,
    BLE_OTA_STATE_BUSY                                       = 0x01,
    BLE_OTA_STATE_FAILED                                     = 0x02,
    BLE_OTA_STATE_TRIAL                                      = 0x03,
    BLE_OTA_STATE_UPDATED                                    = 0x05,
};

class BLEOta : public BLEService {
public:
    BLEOta();
    BLEOta(const BLEOta&) = delete;
    BLEOta& operator=(const BLEOta&) = delete;

    bool begin(uint32_t options = 0);

    uint32_t state();

    void accept();
    void reject();
    void allow();
    void deny();
    void confirm();
    void cancel();

    void onStart(void(*callback)(void));
    void onStart(Callback callback);
    void onFinish(void(*callback)(void));
    void onFinish(Callback callback);
    
    
private:
    uint8_t                     m_busy;
    uint8_t                     m_state;
    uint8_t                     m_status;
    uint8_t                     m_protocol;
    uint8_t                     m_component;
    volatile uint8_t            m_deny;
    uint8_t                     m_options;
    volatile uint8_t            m_sequence;

    uint16_t                    m_head;
    uint16_t                    m_tail;
    uint32_t                    m_offset;
    uint32_t                    m_size;
    uint8_t                     m_data[BLE_OTA_QUEUE_SIZE];

    k_work_t                    m_work;
    stm32wb_fwu_request_t       m_request;

    BLECharacteristic           m_command_characteristic;
    BLECharacteristic           m_event_characteristic;
    BLECharacteristic           m_data_characteristic;

    Callback                    m_start_callback;
    Callback                    m_finish_callback;
    
    static void                 acceptCallback(class BLEOta *self);
    static void                 rejectCallback(class BLEOta *self);
    static void                 beginCallback(class BLEOta *self);
    static void                 startCallback(class BLEOta *self);
    static void                 writeCallback(class BLEOta *self);
    static void                 finishCallback(class BLEOta *self);
    static void                 confirmCallback(class BLEOta *self);
    static void                 rebootCallback();
    static void                 cancelCallback(class BLEOta *self);
    static void                 processCallback(class BLEOta *self);

    void                        connectCallback();
    void                        disconnectCallback();
    void                        commandCallback();
    void                        dataCallback();
    void                        countCallback();
};

#endif // BLE_H

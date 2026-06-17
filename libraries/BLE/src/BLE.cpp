/*
 * BOND_LIST_SIZE  2 (full GATT)
 * BOND_LIST_SIZE  17 (reduced GATT, changed SVC)
 * BOND_LIST_SIZE  22 (reduced GATT, no changed SVC)
 *
 * FILTER_LIST_SIZE    31
 * RESOLVING_LIST_SIZE 15
 */

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

#include "Arduino.h"
#include "BLE.h"
#include "wiring_private.h"

#include "stm32wb_ipcc.h"
#include "stm32wb_boot.h"
#include "stm32wb_fwu.h"

extern "C" {
#include "BLE/ble_core.h"
}

#define BLE_TRACE_SUPPORTED                              0

#define BLE_STATUS_REQUEST_BUSY                          0xff


#define BLE_SLAVE_SCA                                    500
#define BLE_MASTER_SCA                                   0     /* 251 ppm to 500 ppm */

#if defined(STM32WB_CONFIG_BLE_LSE_SOURCE)
#define BLE_LSE_SOURCE                                   STM32WB_CONFIG_BLE_LSE_SOURCE
#else /* STM32WB_CONFIG_BLE_LSE_SOURCE */
#define BLE_LSE_SOURCE                                   BLE_LSE_SOURCE_NOCALIB
#endif /* STM32WB_CONFIG_BLE_LSE_SOURCE */

#if defined(STM32WB_CONFIG_BLE_HSE_STARTUP_TIME)
#define BLE_HSE_STARTUP_TIME                             STM32WB_CONFIG_BLE_HSE_STARTUP_TIME
#else /* STM32WB_CONFIG_BLE_HSE_STARTUP_TIME */
#define BLE_HSE_STARTUP_TIME                             0x148 /* units of 625/256 us (~2.44 us), 800us */
#endif /* STM32WB_CONFIG_BLE_HSE_STARTUP_TIME */

#if defined(STM32WB_CONFIG_BLE_MIN_TX_POWER)
#define BLE_MIN_TX_POWER                                 STM32WB_CONFIG_BLE_MIN_TX_POWER
#else /* STM32WB_CONFIG_BLE_MIN_TX_POWER */
#define BLE_MIN_TX_POWER                                 -40
#endif /* STM32WB_CONFIG_BLE_MIN_TX_POWER */

#if defined(STM32WB_CONFIG_BLE_MAX_TX_POWER)
#define BLE_MAX_TX_POWER                                 STM32WB_CONFIG_BLE_MAX_TX_POWER
#else /* STM32WB_CONFIG_BLE_MAX_TX_POWER */
#define BLE_MAX_TX_POWER                                 6
#endif /* STM32WB_CONFIG_BLE_MAX_TX_POWER */

#if defined(STM32WB_CONFIG_BLE_RX_MODEL)
#define BLE_RX_MODEL                                     STM32WB_CONFIG_BLE_RX_MODEL
#else /* STM32WB_CONFIG_BLE_RX_MODEL */
#define BLE_RX_MODEL                                     BLE_RX_MODEL_AGC_RSSI_LEGACY
#endif /* STM32WB_CONFIG_BLE_RX_MODEL */

#if defined(STM32WB_CONFIG_BLE_TX_PATH_COMPENSATION)
#define BLE_TX_PATH_COMPENSATION                         STM32WB_CONFIG_BLE_TX_PATH_COMPENSATION
#else /* STM32WB_CONFIG_BLE_TX_PATH_COMPENSATION */
#define BLE_TX_PATH_COMPENSATION                         0
#endif /* STM32WB_CONFIG_BLE_TX_PATH_COMPENSATION */

#if defined(STM32WB_CONFIG_BLE_RX_PATH_COMPENSATION)
#define BLE_RX_PATH_COMPENSATION                         STM32WB_CONFIG_BLE_RX_PATH_COMPENSATION
#else /* STM32WB_CONFIG_BLE_RX_PATH_COMPENSATION */
#define BLE_RX_PATH_COMPENSATION                         0
#endif /* STM32WB_CONFIG_BLE_RX_PATH_COMPENSATION */

#define BLE_NUM_LINK                                     4
#define BLE_DATA_LENGTH_EXTENSION                        1
#define BLE_VITERBI_MODE                                 1
#define BLE_MAX_CONN_EVENT_LENGTH                        0xffffffff

#define BLE_NUM_GATT_ATTRIBUTES                          128
#define BLE_NUM_GATT_SERVICES                            16
#define BLE_ATT_VALUE_ARRAY_SIZE                         (10752 - (BLE_NUM_GATT_SERVICES * 48) - (BLE_NUM_GATT_ATTRIBUTES * 40))

#define BLE_PREPARE_WRITE_LIST_SIZE                      (BLE_PREP_WRITE_X_ATT(BLE_MAX_ATT_MTU))
#define BLE_MBLOCK_COUNT                                 (BLE_MBLOCKS_CALC(BLE_PREPARE_WRITE_LIST_SIZE, BLE_MAX_ATT_MTU, BLE_NUM_LINK))

#define BLE_LSE_SOURCE_NOCALIB                           0x00
#define BLE_LSE_SOURCE_CALIB                             0x01
#define BLE_LSE_SOURCE_MOD5MM                            0x02
#define BLE_LSE_SOURCE_HSE                               0x04 /* HSE/1024 vs LSE */

#define BLE_OPTION_LL_ONLY                               0x01
#define BLE_OPTION_NO_SERVICE_CHANGED                    0x02
#define BLE_OPTION_READ_ONLY_DEVICE_NAME                 0x04
#define BLE_OPTION_EXT_ADV                               0x08
#define BLE_OPTION_CS_ALGO2                              0x10
#define BLE_OPTION_REDUCED_GATT_DATABASE_IN_NVM          0x20
#define BLE_OPTION_GATT_CACHING                          0x40
#define BLE_OPTION_POWER_CLASS_2_3                       0x00
#define BLE_OPTION_POWER_CLASS_1                         0x80

#define BLE_RX_MODEL_AGC_RSSI_LEGACY                     0x00
#define BLE_RX_MODEL_AGC_RSSI_BLOCKER                    0x01

#define BLE_CORE_VERSION_5_2                             11 
#define BLE_CORE_VERSION_5_3                             12
#define BLE_CORE_VERSION_5_4                             13

#define BLE_OPTION_EXT_APPEARANCE_WRITEABLE              0x01
#define BLE_OPTION_EXT_ENHANCED_ATT_SUPPORTED            0x02

#define BLE_VALUE_ATTRIB_HANDLE_OFFSET                   1
#define BLE_CCCD_ATTRIB_HANDLE_OFFSET                    2

#define BLE_ATT_SERVICE_HANDLE                           0x0001
#define BLE_SERVICE_CHANGED_HANDLE                       0x0002

#define BLE_GAP_DEFAULT_INTERVAL_MIN                     0x0018 /* 30ms */
#define BLE_GAP_DEFAULT_INTERVAL_MAX                     0x0028 /* 50ms */
#define BLE_GAP_DEFAULT_LATENCY                          0x0000 
#define BLE_GAP_DEFAULT_TIMEOUT                          0x0064 /* 1000ms */
#define BLE_GAP_DEFAULT_MIN_CE_LENGTH                    0x0000
#define BLE_GAP_DEFAULT_MAX_CE_LENGTH                    0x0000

#define BLE_SERVER_STATE_NONE                            0
#define BLE_SERVER_STATE_NOT_READY                       1
#define BLE_SERVER_STATE_READY                           2

#define BLE_BUSY_NONE                                    0
#define BLE_BUSY_HCI_LE_SET_PHY                          1   /* peer */
#define BLE_BUSY_HCI_LE_READ_PHY                         2   /* peer */
#define BLE_BUSY_L2CAP_CONNECTION_PARAMETER_UPDATE       3   /* peer */
#define BLE_BUSY_L2CAP_CONNECTION_PARAMETER_CONFIRM      4   /* peer */
#define BLE_BUSY_GAP_CREATE_CONNECTION                   5   /* peer */
#define BLE_BUSY_GAP_TERMINATE                           6   /* peer */
#define BLE_BUSY_GAP_PASSKEY                             7   /* peer */
#define BLE_BUSY_GAP_PERIPHERAL_SECURITY                 8   /* peer */
#define BLE_BUSY_GAP_CENTRAL_SECURITY                    9   /* peer */
#define BLE_BUSY_GAP_ALLOW_REBOND                        10  /* peer */
#define BLE_BUSY_GATT_NOTIFY_VALUE                       11
#define BLE_BUSY_GATT_SET_VALUE                          12
#define BLE_BUSY_GATT_ALLOW_READ                         13  /* peer */
#define BLE_BUSY_GATT_MTU_EXCHANGE                       14  /* peer */
#define BLE_BUSY_GATT_DISCOVER_SERVICE_BY_UUID           15  /* peer */
#define BLE_BUSY_GATT_DISCOVER_SERVICES                  16  /* peer */
#define BLE_BUSY_GATT_DISCOVER_CHARACTERISTICS           17  /* peer */
#define BLE_BUSY_GATT_DISCOVER_DESCRIPTORS               18  /* peer */
#define BLE_BUSY_GATT_CONFIRM_INDICATION                 19  /* peer */
#define BLE_BUSY_GATT_READ_VALUE                         20  /* peer */
#define BLE_BUSY_GATT_READ_VALUE_LONG                    21  /* peer */
#define BLE_BUSY_GATT_WRITE_VALUE                        22  /* peer */
#define BLE_BUSY_GATT_WRITE_VALUE_LONG                   23  /* peer */
#define BLE_BUSY_GATT_WRITE_VALUE_WITHOUT_RESPONSE       24  /* peer */
#define BLE_BUSY_GATT_WRITE_CONFIG                       25  /* peer */

#define BLE_REQUEST_HCI_LE_SET_PHY                       (1ul << (BLE_BUSY_HCI_LE_SET_PHY -1))
#define BLE_REQUEST_HCI_LE_READ_PHY                      (1ul << (BLE_BUSY_HCI_LE_READ_PHY -1))
#define BLE_REQUEST_L2CAP_CONNECTION_PARAMETER_UPDATE    (1ul << (BLE_BUSY_L2CAP_CONNECTION_PARAMETER_UPDATE -1))
#define BLE_REQUEST_L2CAP_CONNECTION_PARAMETER_CONFIRM   (1ul << (BLE_BUSY_L2CAP_CONNECTION_PARAMETER_CONFIRM -1))
#define BLE_REQUEST_GAP_CREATE_CONNECTION                (1ul << (BLE_BUSY_GAP_CREATE_CONNECTION -1))
#define BLE_REQUEST_GAP_TERMINATE                        (1ul << (BLE_BUSY_GAP_TERMINATE -1))
#define BLE_REQUEST_GAP_PASSKEY                          (1ul << (BLE_BUSY_GAP_PASSKEY -1))
#define BLE_REQUEST_GAP_PERIPHERAL_SECURITY              (1ul << (BLE_BUSY_GAP_PERIPHERAL_SECURITY -1))
#define BLE_REQUEST_GAP_CENTRAL_SECURITY                 (1ul << (BLE_BUSY_GAP_CENTRAL_SECURITY -1))
#define BLE_REQUEST_GAP_ALLOW_REBOND                     (1ul << (BLE_BUSY_GAP_ALLOW_REBOND -1))
#define BLE_REQUEST_GATT_NOTIFY_VALUE                    (1ul << (BLE_BUSY_GATT_NOTIFY_VALUE -1))
#define BLE_REQUEST_GATT_SET_VALUE                       (1ul << (BLE_BUSY_GATT_SET_VALUE -1))
#define BLE_REQUEST_GATT_ALLOW_READ                      (1ul << (BLE_BUSY_GATT_ALLOW_READ -1))
#define BLE_REQUEST_GATT_MTU_EXCHANGE                    (1ul << (BLE_BUSY_GATT_MTU_EXCHANGE -1))
#define BLE_REQUEST_GATT_DISCOVER_SERVICE_BY_UUID        (1ul << (BLE_BUSY_GATT_DISCOVER_SERVICE_BY_UUID -1))
#define BLE_REQUEST_GATT_DISCOVER_SERVICES               (1ul << (BLE_BUSY_GATT_DISCOVER_SERVICES -1))
#define BLE_REQUEST_GATT_DISCOVER_CHARACTERISTICS        (1ul << (BLE_BUSY_GATT_DISCOVER_CHARACTERISTICS -1))
#define BLE_REQUEST_GATT_DISCOVER_DESCRIPTORS            (1ul << (BLE_BUSY_GATT_DISCOVER_DESCRIPTORS -1))
#define BLE_REQUEST_GATT_CONFIRM_INDICATION              (1ul << (BLE_BUSY_GATT_CONFIRM_INDICATION -1))
#define BLE_REQUEST_GATT_READ_VALUE                      (1ul << (BLE_BUSY_GATT_READ_VALUE -1))
#define BLE_REQUEST_GATT_READ_VALUE_LONG                 (1ul << (BLE_BUSY_GATT_READ_VALUE_LONG -1))
#define BLE_REQUEST_GATT_WRITE_VALUE                     (1ul << (BLE_BUSY_GATT_WRITE_VALUE -1))
#define BLE_REQUEST_GATT_WRITE_VALUE_LONG                (1ul << (BLE_BUSY_GATT_WRITE_VALUE_LONG -1))
#define BLE_REQUEST_GATT_WRITE_VALUE_WITHOUT_RESPONSE    (1ul << (BLE_BUSY_GATT_WRITE_VALUE_WITHOUT_RESPONSE -1))
#define BLE_REQUEST_GATT_WRITE_CONFIG                    (1ul << (BLE_BUSY_GATT_WRITE_CONFIG -1))


#define BLE_EVENT_HCI_LE_READ_PHY                         0x0001  /* peer */
#define BLE_EVENT_HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT 0x0002  /* peer */
#define BLE_EVENT_GAP_PAIRING_COMPLETE                    0x0004  /* peer */
#define BLE_EVENT_GATT_MTU_EXCHANGE                       0x0008  /* peer */
#define BLE_EVENT_GATT_TX_POOL_AVAILABLE                  0x0010
#define BLE_EVENT_GATT_SERVER_CONFIRMATION                0x0020
#define BLE_EVENT_GATT_NOTIFICATION_COMPLETE              0x0040
#define BLE_EVENT_GATT_PROC_COMPLETE                      0x0080  /* peer */

#define BLE_PEER_DEVICE_NOT_CONNECTED                    0xffff

#define BLE_TEST_MODE_NONE                               0
#define BLE_TEST_MODE_TRANSMIT                           1
#define BLE_TEST_MODE_RECEIVE                            2

#define BLE_SCAN_HISTORY_ENTRIES                         32
#define BLE_SCAN_QUEUE_ENTRIES                           16

static const void *allocate_noconst(const void *data, uint32_t size) {
    void *_data;
    
    if (((uint32_t)data >= FLASH_BASE) && ((uint32_t)data <= (FLASH_BASE + FLASH_SIZE))) {
        return data;
    }

    _data = malloc(size);

    if (_data) {
        memcpy(_data, data, size);
    }

    return (const void*)_data;
}

static void deallocate_noconst(const void *data) {
    if (!(((uint32_t)data >= FLASH_BASE) && ((uint32_t)data <= (FLASH_BASE + FLASH_SIZE)))) {
        free((void*)data);
    }
}

static void done_callback(void *task) {
    k_event_send((k_task_t *)task, WIRING_EVENT_TRANSIENT);
}

static int ascii2hex(char c) {
    if ((c >= '0') && (c <= '9')) { return ((c - '0') + 0x00); }
    if ((c >= 'a') && (c <= 'f')) { return ((c - 'a') + 0x0a); }
    if ((c >= 'A') && (c <= 'F')) { return ((c - 'A') + 0x0a); }

    return -1;
}

static const char hex2ascii[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };

static const char *uuid_string(const BLEUuid &uuid) {
    static unsigned int slot = 0;
    static char data[4][64];
    char *string;
    int index;

    slot = (slot +1) & 3;
    string = &data[slot][0];
    
    if (uuid.type() == BLE_UUID_TYPE_16) {
        for (index = 1; index >= 0; index--) {
            *string++ = hex2ascii[uuid.data()[index] >> 4];
            *string++ = hex2ascii[uuid.data()[index] & 15];
        }
        
        *string++ = '\0';
        
        return &data[slot][0];
    }
    
    if (uuid.type() == BLE_UUID_TYPE_128) {
        for (index = 15; index >= 0; index--) {
            *string++ = hex2ascii[uuid.data()[index] >> 4];
            *string++ = hex2ascii[uuid.data()[index] & 15];
            
            if ((1u << index) & ((1u << 12) | (1u << 10) | (1u << 8) | (1u << 6))) {
                *string++ = '-';
            }
        }
        
        *string++ = '\0';
        
        return &data[slot][0];
    }

    *string++ = '\0';

    return &data[slot][0];
}

/**********************************************************************************/
/**********************************************************************************
 * CLASS DECLARATIONS
 */

/**********************************************************************************
 * BLE interface
 */

class BLECharacteristicInterface {
public:
    BLECharacteristicInterface() = default;
    virtual ~BLECharacteristicInterface() = default;

    BLECharacteristicInterface(const BLECharacteristicInterface&) = delete;
    BLECharacteristicInterface& operator=(const BLECharacteristicInterface&) = delete;

    virtual operator bool() const;

    inline class BLECharacteristicInterface *interface() { return this; }
    virtual class BLEClientCharacteristic *client();
    virtual class BLEServerCharacteristic *server();

    virtual void reference();
    virtual void unreference();
    
    virtual const BLEUuid uuid() const;
    virtual uint8_t properties() const;
    virtual bool fixedLength() const;

    virtual int valueSize();
    virtual int getValue(void *value, int size);

    virtual bool setValue(const void *value, int length);
    virtual bool notifyValue(const void *value, int length, Callback callback);
    virtual bool writeValue(const void *value, int length, BLEWriteType writeType, Callback callback);

    virtual bool read(Callback callback);
    virtual bool subscribe();
    virtual bool unsubscribe();
  
    virtual uint8_t status();
    virtual bool busy();
    virtual bool written();
    virtual bool subscribed();
    
    virtual void onNotify(Callback callback);
    virtual void onRead(Callback callback);
    virtual void onWrite(Callback callback);
    virtual void onSubscribe(Callback callback);
};

static BLECharacteristicInterface BLENullCharacteristic;

class BLEServiceInterface {
public:
    BLEServiceInterface() = default;
    virtual ~BLEServiceInterface() = default;

    BLEServiceInterface(const BLEServiceInterface&) = delete;
    BLEServiceInterface& operator=(const BLEServiceInterface&) = delete;

    virtual operator bool() const;

    inline class BLEServiceInterface *interface() { return this; }
    virtual class BLEClientService *client();
    virtual class BLEServerService *server();

    virtual void reference();
    virtual void unreference();

    virtual unsigned int characteristicCount();
    virtual BLECharacteristic characteristic(unsigned int index);
    virtual BLECharacteristic characteristic(const BLEUuid &uuid);
    virtual bool addCharacteristic(BLECharacteristic &characteristic);

    virtual const BLEUuid uuid() const;
};

static BLEServiceInterface BLENullService;

class BLEDeviceInterface {
public:
    BLEDeviceInterface() = default;
    virtual ~BLEDeviceInterface() = default;

    BLEDeviceInterface(const BLEDeviceInterface&) = delete;
    BLEDeviceInterface& operator=(const BLEDeviceInterface&) = delete;
  
    virtual operator bool() const;

    inline class BLEDeviceInterface *interface() { return this; }

    virtual void reference();
    virtual void unreference();

    virtual const BLEAddress address() const;
    virtual int mtu();

    virtual void disconnect();
    virtual bool connected();
    
    virtual bool discover(Callback callback);
    virtual bool discoverServices(Callback callback);
    virtual bool discoverCharacteristics(const BLEUuid &uuid, Callback callback);
    virtual bool discoverCharacteristics(BLEService service, Callback callback);
    virtual unsigned int serviceCount();
    virtual BLEService service(unsigned int index);
    virtual BLEService service(const BLEUuid &uuid);
  
    virtual void getPhys(BLEPhy &tx_phy, BLEPhy &rx_phy);
    virtual bool setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout);
    virtual void getConnectionParameters(uint16_t &connectionInterval, uint16_t &latency, uint16_t &timeout); 
    
    virtual void onMTUExchange(Callback callback);
    virtual void onConnectionParameters(Callback callback);
    virtual void onDisconnect(Callback callback);
};

static BLEDeviceInterface BLENullDevice;



/**********************************************************************************
 * GATT client
 */

#define BLE_CLIENT_CONTROL_SEQUENCE               0x80
#define BLE_CLIENT_CONTROL_WRITTEN                0x40
#define BLE_CLIENT_CONTROL_MODIFIED               0x20
#define BLE_CLIENT_CONTROL_CONFIRM                0x10
#define BLE_CLIENT_CONTROL_READ                   0x08
#define BLE_CLIENT_CONTROL_WRITE                  0x04
#define BLE_CLIENT_CONTROL_WRITE_WITHOUT_RESPONSE 0x02
#define BLE_CLIENT_CONTROL_CONFIG                 0x01


class BLEClientCharacteristic final : public BLECharacteristicInterface {
public:
    BLEClientCharacteristic(const BLEUuid &uuid, uint8_t properties);
    ~BLEClientCharacteristic() override;

    operator bool() const override;

    class BLEClientCharacteristic *client() override;
  
    void reference() override;
    void unreference() override;

    const BLEUuid uuid() const override;
    uint8_t properties() const override;

    int valueSize() override;
    int getValue(void *value, int size) override;
  
    bool writeValue(const void *value, int length, BLEWriteType writeType, Callback callback) override;

    bool read(Callback callback) override;
    bool subscribe() override;
    bool unsubscribe() override;
    
    uint8_t status() override;
    bool busy() override;
    bool written() override;
    bool subscribed() override;

    void onNotify(Callback callback) override;
  
private:
    volatile uint16_t m_refcount;

public:
    uint16_t m_handle;
    uint16_t m_value_handle;
    uint16_t m_config_handle;
    class BLEClientCharacteristic *m_sibling;
    class BLEClientService *m_parent;

    const BLEUuid m_uuid;
  
    const uint8_t m_properties : 8;

    volatile uint8_t m_control;
    volatile uint8_t m_status;
    volatile uint8_t m_config;

    uint8_t *m_value_data;
    uint16_t m_value_size;
    uint16_t m_value_length;

    uint8_t *m_write_data;
    uint16_t m_write_size;
    uint16_t m_write_length;
  
    class BLEClientCharacteristic * volatile m_request;
    
    Callback m_done_callback;
    Callback m_notify_callback;
};

class BLEClientService : public BLEServiceInterface {
public:
    BLEClientService(const BLEUuid &uuid);
    ~BLEClientService() override;

    operator bool() const override;

    class BLEClientService *client() override;

    void reference() override;
    void unreference() override;

    unsigned int characteristicCount() override;
    BLECharacteristic characteristic(unsigned int index) override;
    BLECharacteristic characteristic(const BLEUuid &uuid) override;
    
    const BLEUuid uuid() const override;
  
private:
    volatile uint16_t m_refcount;

public:
    uint16_t m_handle;
    uint16_t m_end_group_handle;
    uint16_t m_count;
    class BLEClientCharacteristic *m_children;
    class BLEClientService *m_sibling;
    class BLEPeerDevice *m_parent;

    const BLEUuid m_uuid;
};

/**********************************************************************************
 * GATT Server
 */

/*
 * The core idea is to say that if either WRITTEN or NOTIFY is neither setValue() nor notifyValue()
 * (no internally a remote write) can complete will WRITTEN is cleared by a getValue(), or NOTIFY
 * by the ACI command completion.
 *
 * Nested and out of order setValue() are supported.
 *
 * notifyValue() are blocked till the previous one is complete (including CONFIRM).
 *
 * remote writes are ignored till the previous one has been seen by a getValue().
 *
 * MODIFED means that a remote write completed.
 * WRITTEN acts as a lock for NOTIFY / SYNC (and hence can only be set when BOTH are not set, and MODIFIED is set) 
 * WRITTEN | NOTIFY block setValue(), WRITTEN | NOTIFY | CONFIRM block notifyValue(); notify_current / sync_current block a new request.
 * NOTIFY gets cleared before CONFIRM (later event), so that WRITTEN can be set early.
 * notifyValue() sets NOTIFY or NOTIFY | CONFIRM
 * setValue() sets SYNC | CHANGED. Upon issue (sync_current) CHANGED gets cleared. Upon completion SYNC gets cleared if CHANGED was not set.
 */

#define BLE_SERVER_CONTROL_SEQUENCE      0x80
#define BLE_SERVER_CONTROL_WRITTEN       0x40
#define BLE_SERVER_CONTROL_MODIFIED      0x20
#define BLE_SERVER_CONTROL_NOTIFY        0x04
#define BLE_SERVER_CONTROL_SET           0x02
#define BLE_SERVER_CONTROL_CHANGED       0x01

class BLEServerCharacteristic final : public BLECharacteristicInterface {
public:
    BLEServerCharacteristic(const BLEUuid &uuid, uint8_t properties, uint8_t permissions, uint8_t *value, int valueLength, int valueSize, bool fixedLength, bool constant);
    ~BLEServerCharacteristic() override;

    operator bool() const override;

    class BLEServerCharacteristic *server() override;
  
    void reference() override;
    void unreference() override;

    const BLEUuid uuid() const override;
    uint8_t properties() const override;
    bool fixedLength() const override;

    int valueSize() override;
    int getValue(void *value, int size) override;

    bool setValue(const void *value, int length) override;
    bool notifyValue(const void *value, int length, Callback callback) override;

    uint8_t status() override;
    bool busy() override;
    bool written() override;
    bool subscribed() override;

    void onRead(Callback callback) override;
    void onWrite(Callback callback) override;
    void onSubscribe(Callback callback) override;

private:
    volatile uint16_t m_refcount;
  
public:
    uint16_t m_handle;
    class BLEServerService *m_parent;
    class BLEServerCharacteristic *m_sibling;

    const BLEUuid m_uuid;

    const uint8_t m_properties   : 8;
    const uint8_t m_permissions  : 6;
    const uint8_t m_constant     : 1;
    const uint8_t m_fixed_length : 1;

    volatile uint8_t m_control;
    volatile uint8_t m_status;
    volatile uint16_t m_config;
    
    uint8_t * const m_value_data;
    const uint16_t m_value_size;
    uint16_t m_value_length;

    // m_write_data == &m_value_data[m_value_size]
    // m_write_size == m_value_size
    uint16_t m_write_length;    
    uint16_t m_write_handle;    

    class BLEServerCharacteristic * volatile m_request;

    Callback m_done_callback;
    Callback m_read_callback;
    Callback m_write_callback;
    Callback m_subscribe_callback;
};

class BLEServerService : public BLEServiceInterface {
public:
    BLEServerService(const BLEUuid &uuid);
    ~BLEServerService() override;

    operator bool() const override;

    class BLEServerService *server() override;

    void reference() override;
    void unreference() override;

    unsigned int characteristicCount() override;
    BLECharacteristic characteristic(unsigned int index) override;
    BLECharacteristic characteristic(const BLEUuid &uuid) override;
    bool addCharacteristic(BLECharacteristic &characteristic) override;
    
    const BLEUuid uuid() const override;
  
private:
    volatile uint16_t m_refcount;

public:
    uint16_t m_handle;
    uint16_t m_end_group_handle;
    uint16_t m_count;
    class BLELocalDevice *m_parent;
    class BLEServerService *m_sibling;
    class BLEServerCharacteristic *m_children;
  
    const BLEUuid m_uuid;
};


/**********************************************************************************
 * GAP peer
 */

#define BLE_GATT_PROCEDURE_NONE                                               0
#define BLE_GATT_PROCEDURE_DISCOVER_SERVICE_BY_UUID                           1
#define BLE_GATT_PROCEDURE_DISCOVER_SERVICES                                  2
#define BLE_GATT_PROCEDURE_DISCOVER_CHARACTERISTICS                           3
#define BLE_GATT_PROCEDURE_DISCOVER_DESCRIPTORS                               4
#define BLE_GATT_PROCEDURE_READ_VALUE                                         5
#define BLE_GATT_PROCEDURE_READ_VALUE_LONG                                    6
#define BLE_GATT_PROCEDURE_WRITE_VALUE                                        7
#define BLE_GATT_PROCEDURE_WRITE_VALUE_LONG                                   8
#define BLE_GATT_PROCEDURE_WRITE_VALUE_WITHOUT_RESPONSE                       9
#define BLE_GATT_PROCEDURE_WRITE_CONFIG                                       10

#define BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE                                0
#define BLE_PEER_DEVICE_DISCOVERY_REQUEST_NOT_READY                           1
#define BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER                            2
#define BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_SERVICES                   3
#define BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_UUID    4
#define BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_SERVICE 5

class BLEPeerDevice final : public BLEDeviceInterface {
public:
    BLEPeerDevice();
    ~BLEPeerDevice() override;

    operator bool() const override;
  
    void reference() override;
    void unreference() override;

    const BLEAddress address() const override;
    int mtu() override;

    void disconnect() override;
    bool connected() override;

    bool discover(Callback callback) override;
    bool discoverServices(Callback callback) override;
    bool discoverCharacteristics(const BLEUuid &uuid, Callback callback) override;
    bool discoverCharacteristics(BLEService service, Callback callback) override;
    unsigned int serviceCount() override;
    BLEService service(unsigned int index) override;
    BLEService service(const BLEUuid &uuid) override;
    
    void getPhys(BLEPhy &tx_phy, BLEPhy &rx_phy) override;
    bool setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout) override;
    void getConnectionParameters(uint16_t &interval, uint16_t &latency, uint16_t &timeout) override; 
    
    void onMTUExchange(Callback callback) override;
    void onConnectionParameters(Callback callback) override;
    void onDisconnect(Callback callback) override;
    
private:
    volatile uint16_t m_refcount;

public:
    uint8_t m_index;
    uint16_t m_handle;
    BLEAddress m_address;
    uint8_t m_mtu;
  
    struct {
        uint8_t tx_phy;
        uint8_t rx_phy;
        uint16_t interval;
        uint16_t latency;
        uint16_t timeout;
    } m_connection;

    struct {
        volatile bool discovered;
        uint16_t count;
        BLEClientService *children;
    } m_client;

    struct {
        bool connected;

        uint8_t procedure;
        volatile uint32_t request;
        volatile uint16_t event;

        volatile bool connect_request;
        volatile bool disconnect_request;

        volatile bool params_request;

        struct {
            uint16_t interval_min;
            uint16_t interval_max;
            uint16_t latency;
            uint16_t timeout;
            uint16_t min_ce_length;
            uint16_t max_ce_length;
            uint8_t  sequence;
            uint8_t  accept;
        } params;

        uint32_t passkey;
      
        volatile uint8_t discovery_request;
        BLEUuid discovery_uuid;
        class BLEClientService *discovery_service;
        class BLEClientCharacteristic *discovery_characteristic;

        BLEClientCharacteristic * volatile request_submit;
        BLEClientCharacteristic *request_head;
        BLEClientCharacteristic *request_tail;

        BLEClientCharacteristic *read_current;
        uint16_t read_offset;
        uint8_t read_data[BLE_MAX_ATT_SIZE];
        BLEClientCharacteristic *write_current;

        volatile bool config_request;
        BLEClientCharacteristic *config_current;
      
        volatile bool modified_request;
        volatile bool confirm_request;
    } m_machine;

    k_work_t m_request_work;

    void request();
  
    Callback m_done_callback;
    Callback m_mtu_exchange_callback;
    Callback m_connection_parameters_callback;
    Callback m_disconnect_callback;
};

/**********************************************************************************
 * GAP host
 */

class BLEScanHistoryEntry {
public:  
    uint8_t m_accept       : 1;
    uint8_t m_event_type   : 3;
    uint8_t m_address_type : 2;
    uint8_t m_address[BLE_ADDRESS_SIZE];
};

class BLEScanQueueEntry {
public:  
    uint8_t m_accept       : 1;
    uint8_t m_event_type   : 3;
    uint8_t m_address_type : 2;
    uint8_t m_address[BLE_ADDRESS_SIZE];
    uint8_t m_length;
    uint8_t m_data[BLE_AD_SIZE];
    int8_t  m_rssi;
};

#define BLE_LOCAL_STATE_NONE                                         0
#define BLE_LOCAL_STATE_NOT_READY                                    1
#define BLE_LOCAL_STATE_READY                                        2
#define BLE_LOCAL_STATE_ADVERTISING                                  3
#define BLE_LOCAL_STATE_SCANNING                                     4
#define BLE_LOCAL_STATE_TEST_TRANSMIT                                5
#define BLE_LOCAL_STATE_TEST_RECEIVE                                 6

class BLELocalDevice {
public:
    BLELocalDevice();
    ~BLELocalDevice() { };

    inline bool begin(uint32_t options, int mtu);
    inline void end();
    
    inline const BLEAddress address() const;
    
    inline bool setTxPowerLevel(int txPower);
    inline bool setPreferredPhy(BLEPhy txPhy, BLEPhy rxPhy, BLEPhyOption options);
    inline bool setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout);

    inline bool setDeviceName(const char *deviceName);
    inline bool setAppearance(enum BLEAppearance appearance);

    inline bool setSecurity(BLESecurity security);
    inline bool setPasskey(uint32_t passkey);
    inline bool setBonding(bool bonding);
    inline void setConnectable(bool connectable);
    inline void setDiscoverable(BLEDiscoverable discoverable);
    inline bool setAdvertisingInterval(uint16_t minimumAdvertisingInterval, uint16_t maximumAdvertisingInterval);
    inline bool setAdvertisingData(const BLEAdvertisingData &advertising_data);
    inline void setAdvertisingData();
    inline bool setScanResponseData(const BLEAdvertisingData &advertising_data);
    inline void setScanResponseData();
    inline bool setLocalName(const char *localName);
    inline bool setServiceUuid(const BLEUuid &uuid);
    inline bool setServiceSolicitation(const BLEUuid &uuid);
    inline bool setServiceData(const BLEUuid &uuid, const uint8_t data[], int length);
    inline bool setManufacturerData(const uint8_t data[], int length);
    inline bool setBeacon(const BLEUuid &uuid, uint16_t major, uint16_t minor, int rssi);
    inline bool advertise();
    inline void stopAdvertise();
    inline bool advertising();
    inline BLEDevice central();
    inline bool connected();
    inline void disconnect();

    inline bool setScanInterval(uint16_t scanInterval);
    inline bool setScanWindow(uint16_t scanWindow);
    inline void setScanType(BLEScanType scanType);
    inline bool scan(uint8_t scanMode);
    inline void stopScan();
    inline bool scanning();
    inline unsigned int reportCount();
    inline bool report(BLEAddress &address, int &rssi, BLEScanEvent &event, BLEAdvertisingData &data);
    inline BLEDevice connect(BLEAddress address, uint32_t passkey, Callback callback);
    
    inline bool activate();
    inline unsigned int serviceCount();
    inline BLEService service(unsigned int index);
    inline BLEService service(const BLEUuid &uuid);
    inline bool addService(BLEService &service);

    inline int  queryBondStorage(BLEAddress address[], int count);
    inline bool removeBondStorage(const BLEAddress &address);
    inline bool clearBondStorage();
  
    inline bool testTransmit(int channel, int length, BLEPayload payload,  BLEPhy phy = BLE_PHY_1M);
    inline bool testReceive(int channel, BLEPhy phy = BLE_PHY_1M, int modulation = 0);
    inline uint32_t testStop();
    inline bool testing();

    inline void onStop(Callback callback);
    inline void onReport(Callback callback);
    inline void onConnect(Callback callback);
    inline void onDisconnect(Callback callback);
  
private:
    uint8_t m_state;
    uint8_t m_mtu;
    uint32_t m_options;
    BLEAddress m_address;
    uint8_t m_public_address[6];
    uint8_t m_random_static_address[6];
    uint8_t m_er[16];
    uint8_t m_ir[16];

    struct {
        uint8_t pa_level;
        int8_t tx_power_level;
        uint8_t all_phys;
        uint8_t tx_phy;
        uint8_t rx_phy;
        uint16_t phy_options;
        uint16_t interval_min;
        uint16_t interval_max;
        uint16_t latency;
        uint16_t timeout;
        uint16_t min_ce_length;
        uint16_t max_ce_length;
    } m_connection;
  
    struct {
        const char *device_name;
        uint16_t device_name_length;
        uint16_t appearance;
        uint16_t service_handle;
        uint16_t device_name_handle;
        uint16_t appearance_handle;
        uint16_t connection_parameter_handle;
        bool bonding;
        BLESecurity security;
        uint32_t passkey;
    } m_gap;
  
    struct {
        bool connectable;
        BLEDiscoverable discoverable;
        uint16_t advertising_interval_min;
        uint16_t advertising_interval_max;
        BLEAdvertisingData advertising_data;
        BLEAdvertisingData scan_response_data;
    } m_peripheral;
  
    struct {
        uint16_t scan_interval;
        uint16_t scan_window;
        uint8_t scan_type;
        uint8_t scan_mode;
    } m_central;

    struct {
        volatile bool active;
        uint16_t count;
        class BLEServerService *children;
    } m_server;

    struct {
        volatile uint8_t busy;
        volatile uint32_t request;
        volatile uint16_t event;

        uint8_t index;
        BLEPeerDevice *device;
        
        BLEPeerDevice central;
        BLEPeerDevice *peer[BLE_NUM_LINK];
      
        volatile uint8_t terminate_request;
        k_task_t *terminate_waiting;
      
        BLEServerCharacteristic * volatile request_submit;
        BLEServerCharacteristic *request_head;
        BLEServerCharacteristic *request_tail;

        BLEServerCharacteristic *notify_current;
        uint8_t notify_type;
        uint16_t notify_offset;
        uint16_t notify_count;
        uint16_t notify_length;

        volatile bool set_request;
        BLEServerCharacteristic *set_current;
        uint16_t set_offset;
        uint16_t set_count;
        uint16_t set_length;
        uint8_t set_data[BLE_MAX_ATT_SIZE];

        BLEAdvertisingData advertising_data;
      
        volatile uint8_t report_index;
        volatile uint8_t report_read;
        volatile uint8_t report_write;
        BLEScanHistoryEntry report_history[BLE_SCAN_HISTORY_ENTRIES];
        BLEScanQueueEntry report_queue[BLE_SCAN_QUEUE_ENTRIES];

        stm32wb_ipcc_ble_command_t command;
        uint8_t data[256];
    } m_machine;

    k_work_t m_done_work;
    k_work_t m_event_work;
    k_work_t m_request_work;

    tBleStatus authentication();
    bool setup();
    void teardown();

    void done();
    void event();
    void request();
    void process();
    bool complete(uint16_t handle, uint8_t role, const uint8_t *address, uint8_t type, uint16_t connectionInterval, uint16_t latency, uint16_t timeout);
    void connect(BLEPeerDevice *device);
    void disconnect(BLEPeerDevice *device);

    friend class BLEAdvertisingData;
    friend class BLEServerService;
    friend class BLEServerCharacteristic;
    friend class BLEPeerDevice;

    Callback m_stop_callback;
    Callback m_report_callback;
    Callback m_connect_callback;
    Callback m_disconnect_callback;
};

static BLELocalDevice BLEHost;


/**********************************************************************************/
/**********************************************************************************
 * CLASS IMPLEMENTATIONS
 */

/**********************************************************************************
 * BLE ADDRESS
 */

BLEAddress::BLEAddress() {
    m_type = BLE_ADDRESS_TYPE_NONE;
    memset(&m_data[0], 0, BLE_ADDRESS_SIZE);
}

BLEAddress::BLEAddress(const uint8_t data[BLE_ADDRESS_SIZE], BLEAddressType type) {
    m_type = type;
    memcpy(&m_data[0], &data[0], BLE_ADDRESS_SIZE);
}

BLEAddress::BLEAddress(const char *string, BLEAddressType type) {
    int index, length, data_l, data_h;

    index = -2;
    length = strlen(string);

    m_type = type;
    
    if ((length == 12) || (length == 17)) {
        for (index = 5; index >= 0; index--) {
            data_h = ascii2hex(*string++);
            data_l = ascii2hex(*string++);

            if ((data_h < 0) || (data_l < 0)) {
                break;
            }

            m_data[index] = (data_h << 4) | (data_l << 0);

            if (length == 17) {
                if (index && (*string++ != ':')) {
                    break;
                }
            }
        }
    }
    
    if (index != -1) {
        m_type = BLE_ADDRESS_TYPE_NONE;
        memset(&m_data[0], 0, BLE_ADDRESS_SIZE);
    }
}

BLEAddress::operator bool() const {
    return (m_type != BLE_ADDRESS_TYPE_NONE);
}

bool BLEAddress::operator==(const BLEAddress &other) const {
    if (m_type != other.m_type) {
        return false;
    }
    
    return !memcmp(&m_data[0], &other.m_data[0], BLE_ADDRESS_SIZE);
}

bool BLEAddress::operator!=(const BLEAddress &other) const {
    if (m_type == other.m_type) {
        return false;
    }

    return !!memcmp(&m_data[0], &other.m_data[0], BLE_ADDRESS_SIZE);
}


BLEAddressType BLEAddress::type() const {
    return m_type;
}

const uint8_t* BLEAddress::data() const {
    return m_data;
}

int BLEAddress::toString(char* string, int size, bool stripped) const {
    int index;

    if (m_type != BLE_ADDRESS_TYPE_NONE) {
        if (size >= (stripped ? 13 : 18)) {
            for (index = 5; index >= 0; index--) {
                *string++ = hex2ascii[m_data[index] >> 4];
                *string++ = hex2ascii[m_data[index] & 15];
                
                if (index && !stripped) {
                    *string++ = ':';
                }
            }
            
            *string++ = '\0';
            
            return (stripped ? 12 : 17);
        }
    }
    
    if (size) {
        *string++ = '\0';
    }
    
    return 0;
}

String BLEAddress::toString(bool stripped) const {
    char string[18];

    if (toString(&string[0], sizeof(string), stripped)) {
        return String(&string[0]);
    } else {
        return String();
    }
}


/**********************************************************************************
 * BLE UUID
 */

static const uint8_t ble_uuid128_zeros[BLE_UUID_SIZE] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t ble_uuid128_base[BLE_UUID_SIZE]  = { 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  
BLEUuid::BLEUuid() {
    m_type = BLE_UUID_TYPE_NONE;
    memset(&m_data[0], 0, 16);
}

BLEUuid::BLEUuid(uint16_t data) {
    m_type = BLE_UUID_TYPE_16;
    m_data[0] = data >> 0;
    m_data[1] = data >> 8;
    memset(&m_data[2], 0, 14);
}

BLEUuid::BLEUuid(const uint8_t data[BLE_UUID_SIZE]) {
    if (!memcmp(&data[0], &ble_uuid128_base[0], 12) && !data[14] && !data[15]) {
        m_type = BLE_UUID_TYPE_16;
        m_data[0] = data[12];
        m_data[1] = data[13];
        memset(&m_data[2], 0, 14);
    } else {
        m_type = BLE_UUID_TYPE_128;
        memcpy(&m_data[0], &data[0], BLE_UUID_SIZE);
    }
}

BLEUuid::BLEUuid(const uint8_t data[], BLEUuidType type) {
    m_type = type;
    
    if (type == BLE_UUID_TYPE_16) {
        m_type = BLE_UUID_TYPE_16;
        m_data[0] = data[0];
        m_data[1] = data[1];
        memset(&m_data[2], 0, 14);
    }
    
    if (type == BLE_UUID_TYPE_128) {
        if (!memcmp(&data[0], &ble_uuid128_base[0], 12) && !data[14] && !data[15]) {
            m_type = BLE_UUID_TYPE_16;
            m_data[0] = data[12];
            m_data[1] = data[13];
            memset(&m_data[2], 0, 14);
        } else {
            m_type = BLE_UUID_TYPE_128;
            memcpy(&m_data[0], &data[0], BLE_UUID_SIZE);
        }
    }
}

BLEUuid::BLEUuid(const char *string) {
    int index, length, data_l, data_h;

    index = 0;
    length = strlen(string);

    m_type = BLE_UUID_TYPE_NONE;
    
    if (length == 4) {
        memcpy(&m_data[0], &ble_uuid128_base[0], BLE_UUID_SIZE);

        for (index = 1; index >= 0; index--) {
            data_h = ascii2hex(*string++);
            data_l = ascii2hex(*string++);
            
            if ((data_h < 0) || (data_l < 0)) {
                break;
            }
            
            m_data[index] = (data_h << 4) | (data_l << 0);
        }

        if (index == -1) {
            m_type = BLE_UUID_TYPE_16;
            memset(&m_data[2], 0, 14);
        }
    }

    if ((length == 32) || (length == 36)) {
        for (index = 15; index >= 0; index--) {
            data_h = ascii2hex(*string++);
            data_l = ascii2hex(*string++);
            
            if ((data_h < 0) || (data_l < 0)) {
                break;
            }
            
            m_data[index] = (data_h << 4) | (data_l << 0);
            
            if ((length == 36) && ((1u << index) & ((1u << 12) | (1u << 10) | (1u << 8) | (1u << 6)))) {
                if (*string++ != '-') {
                    break;
                }
            }
        }

        if (index == -1) {
            if (!memcmp(&m_data[0], &ble_uuid128_base[0], 12) && !m_data[14] && !m_data[15]) {
                m_type = BLE_UUID_TYPE_16;
                m_data[0] = m_data[12];
                m_data[1] = m_data[13];
                memset(&m_data[2], 0, 14);
            } else {
                m_type = BLE_UUID_TYPE_128;
            }
        }
    }

    if (m_type == BLE_UUID_TYPE_NONE) {
        memset(&m_data[0], 0, 16);
    }
}

BLEUuid::operator bool() const {
    return (m_type != BLE_UUID_TYPE_NONE);
}

bool BLEUuid::operator==(const BLEUuid &other) const {
    if (m_type != other.m_type) {
        return false;
    }
    
    if (m_type == BLE_UUID_TYPE_16) {
        if ((m_data[0] != other.m_data[0]) || (m_data[1] != other.m_data[1])) {
            return false;
        }
    }
    
    if (m_type == BLE_UUID_TYPE_128) {
        if (memcmp(&m_data[0], &other.m_data[0], 16)) {
            return false;
        }
    }

    return true;
}

bool BLEUuid::operator!=(const BLEUuid &other) const {
    if (m_type != other.m_type) {
        return true;
    }
    
    if (m_type == BLE_UUID_TYPE_16) {
        if ((m_data[0] != other.m_data[0]) || (m_data[1] != other.m_data[1])) {
            return true;
        }
    }
    
    if (m_type == BLE_UUID_TYPE_128) {
        if (memcmp(&m_data[0], &other.m_data[0], 16)) {
            return true;
        }
    }

    return false;
}


BLEUuidType BLEUuid::type() const {
    return m_type;
}

const uint8_t* BLEUuid::data() const {
    return &m_data[0];
}

int BLEUuid::toString(char* string, int size, bool stripped) const {
    int index;

    if (m_type == BLE_UUID_TYPE_16) {
        if (size >= 5) {
            for (index = 1; index >= 0; index--) {
                *string++ = hex2ascii[m_data[index] >> 4];
                *string++ = hex2ascii[m_data[index] & 15];
            }

            *string++ = '\0';
            
            return 4;
        }
    }

    if (m_type == BLE_UUID_TYPE_128) {
        if (size >= (stripped ? 33 : 37)) {
            for (index = 15; index >= 0; index--) {
                *string++ = hex2ascii[m_data[index] >> 4];
                *string++ = hex2ascii[m_data[index] & 15];

                if (!stripped && ((1u << index) & ((1u << 12) | (1u << 10) | (1u << 8) | (1u << 6)))) {
                    *string++ = '-';
                }
            }

            *string++ = '\0';

            return (stripped ? 32 : 36);
        }
    }

    if (size) {
        *string++ = '\0';
    }
    
    return 0;
}

String BLEUuid::toString(bool stripped) const {
    char string[37];

    if (toString(&string[0], sizeof(string), stripped)) {
        return String(&string[0]);
    } else {
        return String();
    }
}

/**********************************************************************************
 * BLE ADVERTISING DATA
 */

BLEAdvertisingData::BLEAdvertisingData() {
    m_length = 0;
}

BLEAdvertisingData::BLEAdvertisingData(BLEAdFlags flags) {
    m_length = 3;

    m_data[0] = 2;
    m_data[1] = BLE_AD_TYPE_FLAGS;
    m_data[2] = flags;
}

BLEAdvertisingData::BLEAdvertisingData(const uint8_t data[], int length) {
    if (length <= BLE_AD_SIZE) {
        m_length = length;

        memcpy(&m_data[0], data, length);
    } else {
        m_length = 0;
    }
}

BLEAdvertisingData::operator bool() const {
    return m_length != 0;
}

int BLEAdvertisingData::length() const {
    return m_length;
}

const uint8_t *BLEAdvertisingData::data() const {
    return &m_data[0];
}

void BLEAdvertisingData::clear() {
    m_length = 0;
}

int BLEAdvertisingData::contains(BLEAdType type) const {
    int index, count;

    if (type == BLE_AD_TYPE_FLAGS) {
        if ((m_length >= 3) && (m_data[0] == 2) && (m_data[1] == BLE_AD_TYPE_FLAGS)) {
            return 3;
        }

        return 0;
    }
    
    for (index = 0; index < m_length; index += count) {
        count = m_data[index +0] +1;

        if (m_data[index +1] == type) {
            return count;
        }
    }

    return 0;
}

bool BLEAdvertisingData::add(BLEAdType type, const uint8_t data[], int length) {
    int index, count;

    if (length == 0) {
        return false;
    }
    
    for (index = 0; index < m_length; index += count) {
        count = m_data[index +0] +1;

        if (m_data[index +1] == type) {
            if ((m_length + (length +2) - count) > BLE_AD_SIZE) {
                return false;
            }

            if (count != (length +2)) {
                memmove(&m_data[index + (length +2) - count], &m_data[index + count], (m_length - (index + count)));

                m_data[index +0] = length +2;
            }

            memcpy(&m_data[index +2], data, length);

            m_length = m_length + (length +2) - count;
            
            return true;
        }
    }

    if ((m_length + (length +2)) > BLE_AD_SIZE) {
        return false;
    }

    if (type == BLE_AD_TYPE_FLAGS) {
        memmove(&m_data[3], &m_data[0], m_length);

        m_data[0] = 2;
        m_data[1] = BLE_AD_TYPE_FLAGS;
        m_data[2] = data[0];
    } else {
        m_data[m_length +0] = length +1;
        m_data[m_length +1] = type;

        memcpy(&m_data[m_length +2], data, length);
    }
    
    m_length += (length +2);

    return true;
}

bool BLEAdvertisingData::remove(BLEAdType type) {
    uint32_t index, count;

    for (index = 0; index < m_length; index += count) {
        count = m_data[index +0] +1;

        if (m_data[index +1] == type) {
            memcpy(&m_data[index +0], &m_data[index + count], (m_length - (index + count)));

            m_length -= count;
            
            return true;
        }
    }

    return false;
}

int BLEAdvertisingData::get(BLEAdType type, uint8_t data[], int size) {
    int index, count, length;
    
    for (index = 0; index < m_length; index += count) {
        count = m_data[index +0] +1;

        if (m_data[index +1] == type) {
            length = count -2;

            if (length > size) {
                length = size;
            }

            memcpy(data, &m_data[index +2], length);

            return length;
        }
    }

    return 0;
}

bool BLEAdvertisingData::setFlags(BLEAdFlags flags) {
    return add(BLE_AD_TYPE_FLAGS, (const uint8_t*)&flags, 1);
}

bool BLEAdvertisingData::setLocalName(const char *local_name) {
  //#warning allow a complete/incomplete setting here, somehow ?  
    if (local_name) {
        return add(BLE_AD_TYPE_COMPLETE_LOCAL_NAME, (const uint8_t*)local_name, strlen(local_name));
    }

    remove(BLE_AD_TYPE_COMPLETE_LOCAL_NAME);

    return true;
}

bool BLEAdvertisingData::setAppearance(BLEAppearance appearance) {
    uint8_t data[2];

    data[0] = appearance & 0xff;
    data[1] = appearance >> 8;

    return add(BLE_AD_TYPE_APPEARANCE, data, 2);
}

bool BLEAdvertisingData::setTxPowerLevel(int tx_power_level) {
    uint8_t data[1];

    ((int8_t*)data)[0] = tx_power_level;

    return add(BLE_AD_TYPE_TX_POWER_LEVEL, data, 1);
}

bool BLEAdvertisingData::setConnectionInterval(uint16_t interval_min, uint16_t interval_max) {
    uint8_t data[4];

    data[0] = interval_min & 0xff;
    data[1] = interval_min >> 8;
    data[2] = interval_max & 0xff;
    data[3] = interval_max >> 8;

    return add(BLE_AD_TYPE_PERIPHERAL_CONN_INTERVAL, data, 4);
}

bool BLEAdvertisingData::setServiceUuid(const BLEUuid &uuid) {
    int size;

    if (uuid) {
        if (uuid.type() == BLE_UUID_TYPE_16) {
            size = 2;
        } else {
            size = 16;
        }

        size += 2;
        size -= contains(BLE_AD_TYPE_16_BIT_SERV_UUID);
        size -= contains(BLE_AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST);
        size -= contains(BLE_AD_TYPE_128_BIT_SERV_UUID);
        size -= contains(BLE_AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST);

        if ((m_length + size) > BLE_MAX_AD_LENGTH) {
            return false;
        }

        if (uuid.type() == BLE_UUID_TYPE_16) {
            remove(BLE_AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST);
            remove(BLE_AD_TYPE_128_BIT_SERV_UUID);
            remove(BLE_AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST);
            return add(BLE_AD_TYPE_16_BIT_SERV_UUID, uuid.data(), 2);
        } else {
            remove(BLE_AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST);
            remove(BLE_AD_TYPE_16_BIT_SERV_UUID);
            remove(BLE_AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST);
            return add(BLE_AD_TYPE_128_BIT_SERV_UUID, uuid.data(), 16);
        }
    }
    
    return false;
}

bool BLEAdvertisingData::setServiceUuid(const char *uuid) {
    return setServiceUuid(BLEUuid(uuid));
}

bool BLEAdvertisingData::setServiceSolicitation(const BLEUuid &uuid) {
    int size;

    if (uuid) {
        if (uuid.type() == BLE_UUID_TYPE_16) {
            size = 2;
        } else {
            size = 16;
        }

        size += 2;
        size -= contains(BLE_AD_TYPE_SERV_SOLICIT_16_BIT_UUID_LIST);
        size -= contains(BLE_AD_TYPE_SERV_SOLICIT_128_BIT_UUID_LIST);

        if ((m_length + size) > BLE_MAX_AD_LENGTH) {
            return false;
        }

        if (uuid.type() == BLE_UUID_TYPE_16) {
            remove(BLE_AD_TYPE_SERV_SOLICIT_128_BIT_UUID_LIST);
            return add(BLE_AD_TYPE_SERV_SOLICIT_16_BIT_UUID_LIST, uuid.data(), 2);
        } else {
            remove(BLE_AD_TYPE_SERV_SOLICIT_16_BIT_UUID_LIST);
            return add(BLE_AD_TYPE_SERV_SOLICIT_128_BIT_UUID_LIST, uuid.data(), 16);
        }
    }
    
    return false;
}

bool BLEAdvertisingData::setServiceSolicitation(const char *uuid) {
    return setServiceSolicitation(BLEUuid(uuid));
}

bool BLEAdvertisingData::setServiceData(const BLEUuid &uuid, const uint8_t data[], int length) {
    uint8_t ad_data[BLE_AD_SIZE];
    int size;
    
    if (uuid) {
        if (uuid.type() == BLE_UUID_TYPE_16) {
            size = 2;
        } else {
            size = 16;
        }

        size += (2 + length);
        size -= contains(BLE_AD_TYPE_SERVICE_DATA_16_BIT_UUID);
        size -= contains(BLE_AD_TYPE_SERVICE_DATA_128_BIT_UUID);

        if ((m_length + size) > BLE_MAX_AD_LENGTH) {
            return false;
        }

        if (uuid.type() == BLE_UUID_TYPE_16) {
            remove(BLE_AD_TYPE_SERVICE_DATA_128_BIT_UUID);

            memcpy(&ad_data[0], uuid.data(), 2);
            memcpy(&ad_data[2], &data[0], length);

            return add(BLE_AD_TYPE_SERVICE_DATA_16_BIT_UUID, ad_data, (2 + length));
        } else {
            remove(BLE_AD_TYPE_SERVICE_DATA_16_BIT_UUID);

            memcpy(&ad_data[0], uuid.data(), 16);
            memcpy(&ad_data[16], &data[0], length);

            return add(BLE_AD_TYPE_SERVICE_DATA_128_BIT_UUID, ad_data, (16 + length));
        }
    }

    return false;
}

bool BLEAdvertisingData::setServiceData(const char *uuid, const uint8_t data[], int length) {
    return setServiceData(BLEUuid(uuid), data, length);
}

bool BLEAdvertisingData::setManufacturerData(const uint8_t data[], int length) {
    return add(BLE_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, data, length);
}

bool BLEAdvertisingData::setBeacon(const BLEUuid&uuid, uint16_t major, uint16_t minor, int rssi) {
    uint8_t ad_data[BLE_AD_SIZE];
    int index;

    if (uuid.type() != BLE_UUID_TYPE_128) {
        return false;
    }

    ad_data[0] = 0x4c;
    ad_data[1] = 0x00;
    ad_data[2] = 0x02;
    ad_data[3] = 0x15;
    
    for (index = 0; index < 16; index++) {
        ad_data[index +4] = uuid.data()[15 - index];
    }

    ad_data[20] = (uint8_t)(major >> 8);
    ad_data[21] = (uint8_t)(major >> 0);
    ad_data[22] = (uint8_t)(minor >> 8);
    ad_data[23] = (uint8_t)(minor >> 0);

    ((int8_t*)ad_data)[24] = rssi;
    
    return add(BLE_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, ad_data, 25);
}

bool BLEAdvertisingData::setBeacon(const char *uuid, uint16_t major, uint16_t minor, int rssi) {
    return setBeacon(BLEUuid(uuid), major, minor, rssi);
}

bool BLEAdvertisingData::getFlags(BLEAdFlags &flags) {
    uint8_t data[1];

    if (get(BLE_AD_TYPE_FLAGS, &data[0], sizeof(data)) == sizeof(data)) {
        flags = (BLEAdFlags)data[0];

        return true;
    }

    return false;
}

bool BLEAdvertisingData::getTxPowerLevel(int &tx_power_level) {
    uint8_t data[1];

    if (get(BLE_AD_TYPE_TX_POWER_LEVEL, &data[0], sizeof(data)) == sizeof(data)) {
        tx_power_level = ((int8_t*)&data[0])[0];

        return true;
    }

    return false;
}

bool BLEAdvertisingData::getConnectionInterval(uint16_t &interval_min, uint16_t &interval_max) {
    uint8_t data[4];

    if (get(BLE_AD_TYPE_PERIPHERAL_CONN_INTERVAL, &data[0], sizeof(data)) == sizeof(data)) {
        interval_min = (data[1] << 8) | data[0];
        interval_max = (data[3] << 8) | data[1];

        return true;
    }

    return false;
}

bool BLEAdvertisingData::getLocalName(char *local_name, int size) {
    int length;

    if (!size) {
        return false;
    }
    
    length = get(BLE_AD_TYPE_COMPLETE_LOCAL_NAME, (uint8_t*)local_name, (size -1));

    if (!length) {
        length = get(BLE_AD_TYPE_SHORTENED_LOCAL_NAME, (uint8_t*)local_name, (size -1));
    }

    if (length) {
        local_name[length] = '\0';
        return true;
    }
    
    return false;
}

bool BLEAdvertisingData::getAppearance(BLEAppearance &appearance) {
    uint8_t data[2];

    if (get(BLE_AD_TYPE_APPEARANCE, &data[0], sizeof(data)) == sizeof(data)) {
        appearance = (BLEAppearance)((data[1] << 8) | data[0]);

        return true;
    }

    return false;
}

bool BLEAdvertisingData::getServiceUuid(const char *uuid) {
    return getServiceUuid(BLEUuid(uuid));
}

bool BLEAdvertisingData::getServiceUuid(const BLEUuid &uuid) {
    uint8_t ad_data[BLE_AD_SIZE];
    int ad_length, ad_offset;

    
    ad_length = get(BLE_AD_TYPE_16_BIT_SERV_UUID, &ad_data[0], sizeof(ad_data));

    for (ad_offset = 0; ad_offset < ad_length; ad_offset += 2) {
        if (uuid == BLEUuid(&ad_data[ad_offset], BLE_UUID_TYPE_16)) {
            return true;
        }
    }

    ad_length = get(BLE_AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST, &ad_data[0], sizeof(ad_data));

    for (ad_offset = 0; ad_offset < ad_length; ad_offset += 2) {
        if (uuid == BLEUuid(&ad_data[ad_offset], BLE_UUID_TYPE_16)) {
            return true;
        }
    }

    ad_length = get(BLE_AD_TYPE_128_BIT_SERV_UUID, &ad_data[0], sizeof(ad_data));

    for (ad_offset = 0; ad_offset < ad_length; ad_offset += 16) {
        if (uuid == BLEUuid(&ad_data[ad_offset], BLE_UUID_TYPE_128)) {
            return true;
        }
    }

    ad_length = get(BLE_AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST, &ad_data[0], sizeof(ad_data));

    for (ad_offset = 0; ad_offset < ad_length; ad_offset += 16) {
        if (uuid == BLEUuid(&ad_data[ad_offset], BLE_UUID_TYPE_128)) {
            return true;
        }
    }
    
    return false;
}

bool BLEAdvertisingData::getServiceUuids(BLEUuid uuid[], int size, int &count) {
    uint8_t ad_data[BLE_AD_SIZE];
    int ad_length, ad_offset, index;

    index = 0;
    
    ad_length = get(BLE_AD_TYPE_16_BIT_SERV_UUID, &ad_data[0], sizeof(ad_data));

    for (ad_offset = 0; ad_offset < ad_length; ad_offset += 2) {
        if (index < size) {
            uuid[index++] = BLEUuid(&ad_data[ad_offset], BLE_UUID_TYPE_16);
        }
    }

    ad_length = get(BLE_AD_TYPE_16_BIT_SERV_UUID_CMPLT_LIST, &ad_data[0], sizeof(ad_data));

    for (ad_offset = 0; ad_offset < ad_length; ad_offset += 2) {
        if (index < size) {
            uuid[index++] = BLEUuid(&ad_data[ad_offset], BLE_UUID_TYPE_16);
        }
    }

    ad_length = get(BLE_AD_TYPE_128_BIT_SERV_UUID, &ad_data[0], sizeof(ad_data));

    for (ad_offset = 0; ad_offset < ad_length; ad_offset += 16) {
        if (index < size) {
            uuid[index++] = BLEUuid(&ad_data[ad_offset], BLE_UUID_TYPE_128);
        }
    }

    ad_length = get(BLE_AD_TYPE_128_BIT_SERV_UUID_CMPLT_LIST, &ad_data[0], sizeof(ad_data));

    for (ad_offset = 0; ad_offset < ad_length; ad_offset += 16) {
        if (index < size) {
            uuid[index++] = BLEUuid(&ad_data[ad_offset], BLE_UUID_TYPE_128);
        }
    }

    if (index) {
        count = index;

        return true;
    }
    
    return false;
}

bool BLEAdvertisingData::getServiceSolicitation(BLEUuid uuid[], int size, int &count) {
    uint8_t ad_data[BLE_AD_SIZE];
    int ad_length, ad_offset, index;

    index = 0;
    
    ad_length = get(BLE_AD_TYPE_SERV_SOLICIT_16_BIT_UUID_LIST, &ad_data[0], sizeof(ad_data));

    for (ad_offset = 0; ad_offset < ad_length; ad_offset += 2) {
        if (index < size) {
            uuid[index++] = BLEUuid(&ad_data[ad_offset], BLE_UUID_TYPE_16);
        }
    }

    ad_length = get(BLE_AD_TYPE_SERV_SOLICIT_128_BIT_UUID_LIST, &ad_data[0], sizeof(ad_data));

    for (ad_offset = 0; ad_offset < ad_length; ad_offset += 16) {
        if (index < size) {
            uuid[index++] = BLEUuid(&ad_data[ad_offset], BLE_UUID_TYPE_128);
        }
    }

    if (index) {
        count = index;

        return true;
    }
    
    return false;
}

bool BLEAdvertisingData::getServiceData(BLEUuid &uuid, uint8_t data[], int size, int &length) {
    uint8_t ad_data[BLE_AD_SIZE];
    int ad_length;

    ad_length = get(BLE_AD_TYPE_SERVICE_DATA_16_BIT_UUID, &ad_data[0], sizeof(ad_data));

    if (ad_length >= 2) {
        uuid = BLEUuid(&ad_data[0], BLE_UUID_TYPE_16);

        if ((ad_length - 2) < size) {
            size = (ad_length - 2);
        }

        memcpy(&data[0], &ad_data[2], size);

        length = size;

        return true;
    }

    ad_length = get(BLE_AD_TYPE_SERVICE_DATA_128_BIT_UUID, &ad_data[0], sizeof(ad_data));

    if (ad_length >= 16) {
        uuid = BLEUuid(&ad_data[0], BLE_UUID_TYPE_128);

        if ((ad_length - 16) < size) {
            size = (ad_length - 16);
        }

        memcpy(&data[0], &ad_data[16], size);

        length = size;

        return true;
    }
    
    return false;
}

bool BLEAdvertisingData::getManufacturerData(uint8_t data[], int size, int &length) {
    uint8_t ad_data[BLE_AD_SIZE];
    int ad_length;

    ad_length = get(BLE_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, &ad_data[0], sizeof(ad_data));

    if (ad_length && (ad_length <= size)) {
        if (!((ad_length == 25) && ((ad_data[0] == 0x4c) && (ad_data[1] == 0x00) && (ad_data[2] == 0x02) && (ad_data[3] == 0x15)))) {
            memcpy(&data[0], &ad_data[0], ad_length);

            length = ad_length;
            
            return true;
        }
    }

    return false;
}

bool BLEAdvertisingData::getBeacon(BLEUuid &uuid, uint16_t &major, uint16_t &minor, int &rssi) {
    uint8_t ad_data[BLE_AD_SIZE], uuid_data[BLE_UUID_SIZE];
    int length, index;

    length = get(BLE_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, &ad_data[0], sizeof(ad_data));

    if ((length == 25) && ((ad_data[0] == 0x4c) && (ad_data[1] == 0x00) && (ad_data[2] == 0x02) && (ad_data[3] == 0x15))) {
        for (index = 0; index < 16; index++) {
            uuid_data[15 - index] = ad_data[index +4];
        }

        uuid = BLEUuid(uuid_data);

        major = (ad_data[21] << 8) | ad_data[20];
        minor = (ad_data[23] << 8) | ad_data[22];
        rssi = ((int8_t*)ad_data)[24];

        return true;
    }

    return false;
}

/**********************************************************************************
 * BLE interface
 */

BLECharacteristicInterface::operator bool() const {
    return false;
}

class BLEClientCharacteristic *BLECharacteristicInterface::client() {
    return nullptr;
}

class BLEServerCharacteristic *BLECharacteristicInterface::server() {
    return nullptr;
}

void BLECharacteristicInterface::reference() {
}

void BLECharacteristicInterface::unreference() {
}

const BLEUuid BLECharacteristicInterface::uuid() const {
    return BLEUuid();
}

uint8_t BLECharacteristicInterface::properties() const {
    return 0;
}

bool BLECharacteristicInterface::fixedLength() const {
    return false;
}

int BLECharacteristicInterface::valueSize() {
    return 0;
}


int BLECharacteristicInterface::getValue(void *value __attribute__((unused)), int size __attribute__((unused))) {
    return 0;
}

bool BLECharacteristicInterface::setValue(const void *value __attribute__((unused)), int length __attribute__((unused))) {
    return false;
}

bool BLECharacteristicInterface::notifyValue(const void *value __attribute__((unused)), int length __attribute__((unused)), Callback callback __attribute__((unused))) {
    return false;
}

bool BLECharacteristicInterface::writeValue(const void *value __attribute__((unused)), int length __attribute__((unused)), BLEWriteType writeType __attribute__((unused)), Callback callback __attribute__((unused))) {
    return false;
}

bool BLECharacteristicInterface::read(Callback callback __attribute__((unused))) {
    return false;
}

bool BLECharacteristicInterface::subscribe() {
    return false;
}

bool BLECharacteristicInterface::unsubscribe() {
    return false;
}

uint8_t BLECharacteristicInterface::status() {
    return BLE_STATUS_SUCCESS;
}

bool BLECharacteristicInterface::busy() {
    return false;
}

bool BLECharacteristicInterface::written() {
    return false;
}

bool BLECharacteristicInterface::subscribed() {
    return false;
}

void BLECharacteristicInterface::onNotify(Callback callback __attribute__((unused))) {
}

void BLECharacteristicInterface::onRead(Callback callback __attribute__((unused))) {
}

void BLECharacteristicInterface::onWrite(Callback callback __attribute__((unused))) {
}

void BLECharacteristicInterface::onSubscribe(Callback callback __attribute__((unused))) {
}


BLEServiceInterface::operator bool() const {
    return false;
}

class BLEClientService *BLEServiceInterface::client() {
    return nullptr;
}

class BLEServerService *BLEServiceInterface::server() {
    return nullptr;
}

void BLEServiceInterface::reference() {
}

void BLEServiceInterface::unreference() {
}

unsigned int BLEServiceInterface::characteristicCount() {
    return 0;
}

BLECharacteristic BLEServiceInterface::characteristic(unsigned int index __attribute__((unused))) {
    return BLECharacteristic();
}

BLECharacteristic BLEServiceInterface::characteristic(const BLEUuid &uuid __attribute__((unused))) {
    return BLECharacteristic();
}

bool BLEServiceInterface::addCharacteristic(BLECharacteristic &characteristic __attribute__((unused))) {
    return false;
}

const BLEUuid BLEServiceInterface::uuid() const {
    return BLEUuid();
}

BLEDeviceInterface::operator bool() const {
    return false;
}

void BLEDeviceInterface::reference() {
}

void BLEDeviceInterface::unreference() {
}

const BLEAddress BLEDeviceInterface::address() const {
    return BLEAddress();
}

int BLEDeviceInterface::mtu() {
    return 0;
}

bool BLEDeviceInterface::connected() {
    return false;
}

void BLEDeviceInterface::disconnect() {
}

bool BLEDeviceInterface::discover(Callback callback __attribute__((unused))) {
    return false;
}

bool BLEDeviceInterface::discoverServices(Callback callback __attribute__((unused))) {
    return false;
}

bool BLEDeviceInterface::discoverCharacteristics(const BLEUuid &uuid __attribute__((unused)), Callback callback __attribute__((unused))) {
    return false;
}

bool BLEDeviceInterface::discoverCharacteristics(BLEService service __attribute__((unused)), Callback callback __attribute__((unused))) {
    return false;
}

unsigned int BLEDeviceInterface::serviceCount() {
    return 0;
}

BLEService BLEDeviceInterface::service(unsigned int index __attribute__((unused))) {
    return BLEService();
}

BLEService BLEDeviceInterface::service(const BLEUuid &uuid __attribute__((unused))) {
    return BLEService();
}

void BLEDeviceInterface::getPhys(BLEPhy &tx_phy, BLEPhy &rx_phy) {
    tx_phy = (BLEPhy)0;
    rx_phy = (BLEPhy)0;
}

bool BLEDeviceInterface::setConnectionParameters(uint16_t interval_min __attribute__((unused)), uint16_t interval_max __attribute__((unused)), uint16_t latency __attribute__((unused)), uint16_t timeout __attribute__((unused))) {
    return false;
}

void BLEDeviceInterface::getConnectionParameters(uint16_t &interval, uint16_t &latency, uint16_t &timeout) {
    interval = 0;
    latency = 0;
    timeout = 0;
}

void BLEDeviceInterface::onMTUExchange(Callback callback __attribute__((unused))) {
}

void BLEDeviceInterface::onConnectionParameters(Callback callback __attribute__((unused))) {
}

void BLEDeviceInterface::onDisconnect(Callback callback __attribute__((unused))) {
}


/**********************************************************************************
 * GATT client
 */


BLEClientCharacteristic::BLEClientCharacteristic(const BLEUuid &uuid, uint8_t properties) :
    m_uuid(uuid),
    m_properties(properties)
{
    m_refcount = 1;

    m_handle = 0;
    m_value_handle = 0;
    m_config_handle = 0;
    m_parent = nullptr;
    m_sibling = nullptr;

    m_control = 0;
    m_status = BLE_STATUS_SUCCESS;
    m_config = 0;

    m_value_data =  nullptr;
    m_value_size = 0;
    m_value_length = 0;

    m_write_data =  nullptr;
    m_write_size = 0;
    m_write_length = 0;
}

BLEClientCharacteristic::~BLEClientCharacteristic() {
    if (m_value_data) {
        free(m_value_data);
    }

    if (m_write_data) {
        free(m_write_data);
    }
}

BLEClientCharacteristic::operator bool() const {
    return true;
}

class BLEClientCharacteristic *BLEClientCharacteristic::client() {
    return this;
}
  
void BLEClientCharacteristic::reference() {
    __armv7m_atomic_inch(&m_refcount, 0xffff);
}

void BLEClientCharacteristic::unreference() {
    if (__armv7m_atomic_dech(&m_refcount) == 1) {
        delete this;
    }
}

const BLEUuid BLEClientCharacteristic::uuid() const {
    return m_uuid;
}

uint8_t BLEClientCharacteristic::properties() const {
    return m_properties;
}

int BLEClientCharacteristic::valueSize() {
    return m_value_size;
}

int BLEClientCharacteristic::getValue(void *value, int size) {
    BLEPeerDevice *device;
    uint8_t control;
    int length;

    do {
        armv7m_atomic_orb(&m_control, BLE_SERVER_CONTROL_SEQUENCE);
      
        length = size;
        
        if (length > m_value_length) {
            length = m_value_length;
        }
        
        memcpy(value, &m_value_data[0], length);
    } while (!(m_control & BLE_SERVER_CONTROL_SEQUENCE));

    control = armv7m_atomic_andb(&m_control, ~(BLE_CLIENT_CONTROL_READ | BLE_CLIENT_CONTROL_SEQUENCE));

    if ((control & (BLE_CLIENT_CONTROL_MODIFIED | BLE_CLIENT_CONTROL_WRITTEN)) == BLE_CLIENT_CONTROL_MODIFIED) {
        device = m_parent->m_parent;

        device->m_machine.modified_request = true;

        k_work_submit(&device->m_request_work);
    } else {
        control = armv7m_atomic_andb(&m_control, ~(BLE_CLIENT_CONTROL_CONFIRM | BLE_CLIENT_CONTROL_MODIFIED | BLE_CLIENT_CONTROL_WRITTEN));

        if (control & BLE_CLIENT_CONTROL_CONFIRM) {
            device = m_parent->m_parent;
            
            device->m_machine.confirm_request = true;
            
            k_work_submit(&device->m_request_work);
        }
    }

    return length;
}

bool BLEClientCharacteristic::writeValue(const void *value, int length, BLEWriteType writeType, Callback callback) {
    BLEClientCharacteristic *request_submit;
    BLEPeerDevice *device;
    uint8_t control;

    if (writeType == BLE_WRITE_TYPE_DEFAULT) {
        if (m_properties & BLE_PROPERTY_WRITE) {
            writeType = BLE_WRITE_TYPE_WITH_RESPONSE;
        }
        else if (m_properties & BLE_PROPERTY_WRITE_WITHOUT_RESPONSE) {
            writeType = BLE_WRITE_TYPE_WITHOUT_RESPONSE;
        } else {
            return false;
        }
    } else if (writeType == BLE_WRITE_TYPE_WITH_RESPONSE) {
        if (!(m_properties & BLE_PROPERTY_WRITE)) {
            return false;
        }
    } else if (writeType == BLE_WRITE_TYPE_WITHOUT_RESPONSE) {
        if (!(m_properties & BLE_PROPERTY_WRITE_WITHOUT_RESPONSE)) {
            return false;
        }
    }

    do {
        control = m_control;

        if (control & (BLE_CLIENT_CONTROL_WRITTEN | BLE_CLIENT_CONTROL_READ | BLE_CLIENT_CONTROL_WRITE)) {
            return false;
        }
    } while(armv7m_atomic_casb(&m_control, control, (control | BLE_CLIENT_CONTROL_WRITE)) != control);

    if (writeType == BLE_WRITE_TYPE_WITHOUT_RESPONSE) {
        armv7m_atomic_orb(&m_control, BLE_CLIENT_CONTROL_WRITE_WITHOUT_RESPONSE);
    }

    m_value_length = 0;
    
    if (m_value_size < length) {
        m_value_data = (uint8_t*)realloc(m_value_data, length);
        
        if (m_value_data) {
            m_value_size = length;
        } else {
            m_value_size = 0;
        }
    }

    if (!m_value_size) {
        armv7m_atomic_andb(&m_control, ~(BLE_CLIENT_CONTROL_WRITE | BLE_CLIENT_CONTROL_WRITE_WITHOUT_RESPONSE));

        return false;
    }

    memcpy(&m_value_data[0], value, length);

    m_value_length = length;

    armv7m_atomic_andb(&m_control, ~BLE_CLIENT_CONTROL_SEQUENCE);
    
    m_done_callback = callback;

    m_status = BLE_STATUS_BUSY;
    
    device = m_parent->m_parent;
    
    do {
        request_submit = device->m_machine.request_submit;

        m_request = request_submit;
    } while ((BLEClientCharacteristic *)armv7m_atomic_cas((volatile uint32_t*)&device->m_machine.request_submit, (uint32_t)request_submit, (uint32_t)this) != request_submit);

    k_work_submit(&device->m_request_work);

    return true;
}

bool BLEClientCharacteristic::read(Callback callback) {
    BLEClientCharacteristic *request_submit;
    BLEPeerDevice *device;
    uint8_t control;

    if (!(m_properties & BLE_PROPERTY_READ)) {
        return false;
    }

    do {
        control = m_control;

        if (control & (BLE_CLIENT_CONTROL_WRITTEN | BLE_CLIENT_CONTROL_READ | BLE_CLIENT_CONTROL_WRITE)) {
            return false;
        }
    } while(armv7m_atomic_casb(&m_control, control, (control | BLE_CLIENT_CONTROL_READ)) != control);

    m_done_callback = callback;

    m_status = BLE_STATUS_BUSY;
    
    device = m_parent->m_parent;
    
    do {
        request_submit = device->m_machine.request_submit;

        m_request = request_submit;
    } while ((BLEClientCharacteristic *)armv7m_atomic_cas((volatile uint32_t*)&device->m_machine.request_submit, (uint32_t)request_submit, (uint32_t)this) != request_submit);

    k_work_submit(&device->m_request_work);

    return true;
}

bool BLEClientCharacteristic::subscribe() {
    BLEPeerDevice *device;

    if (!(m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE))) {
        return false;
    }
    
    m_config = (m_properties & BLE_PROPERTY_INDICATE) ? 0x02 : 0x01;
    
    armv7m_atomic_orb(&m_control, BLE_CLIENT_CONTROL_CONFIG);
    
    device = m_parent->m_parent;

    device->m_machine.config_request = true;
    
    k_work_submit(&device->m_request_work);

    return true;
}

bool BLEClientCharacteristic::unsubscribe() {
    BLEPeerDevice *device;

    if (!(m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE))) {
        return false;
    }

    m_config = 0;

    armv7m_atomic_orb(&m_control, BLE_CLIENT_CONTROL_CONFIG);
    
    device = m_parent->m_parent;

    device->m_machine.config_request = true;
    
    k_work_submit(&device->m_request_work);

    return true;
}

uint8_t BLEClientCharacteristic::status() {
    return m_status;
}

bool BLEClientCharacteristic::busy() {
    return (m_status == BLE_STATUS_BUSY);
}

bool BLEClientCharacteristic::written() {
    return !!(m_control & BLE_CLIENT_CONTROL_WRITTEN);
}


bool BLEClientCharacteristic::subscribed() {
    return !!m_config;
}

void BLEClientCharacteristic::onNotify(Callback callback) {
    m_notify_callback = callback;
}

BLEClientService::BLEClientService(const BLEUuid &uuid) :
    m_uuid(uuid)
{
    m_refcount = 1;

    m_count = 0;
    m_handle = 0;
    m_end_group_handle = 0;
    m_parent = nullptr;
    m_sibling = nullptr;
    m_children = nullptr;
}

BLEClientService::~BLEClientService() {
}

BLEClientService::operator bool() const {
    return true;
}

class BLEClientService *BLEClientService::client() {
    return this;
}

void BLEClientService::reference() {
    __armv7m_atomic_inch(&m_refcount, 0xffff);
}

void BLEClientService::unreference() {
    if (__armv7m_atomic_dech(&m_refcount) == 1) {
        delete this;
    }
}

unsigned int BLEClientService::characteristicCount() {
    return m_count;
}

BLECharacteristic BLEClientService::characteristic(unsigned int index) {
    class BLEClientCharacteristic *characteristicCurrent;

    for (characteristicCurrent = m_children; characteristicCurrent; characteristicCurrent = characteristicCurrent->m_sibling, index--) {
        if (index == 0) {
            return BLECharacteristic(characteristicCurrent);
        }
    }
    
    return BLECharacteristic();
}

BLECharacteristic BLEClientService::characteristic(const BLEUuid &uuid) {
    class BLEClientCharacteristic *characteristicCurrent;

    if (uuid) {
        for (characteristicCurrent = m_children; characteristicCurrent; characteristicCurrent = characteristicCurrent->m_sibling) {
            if (uuid == characteristicCurrent->m_uuid) {
                return BLECharacteristic(characteristicCurrent);
            }
        }
    }
    
    return BLECharacteristic();
}

const BLEUuid BLEClientService::uuid() const {
    return m_uuid;
}


/**********************************************************************************
 * GATT server
 */

BLEServerCharacteristic::BLEServerCharacteristic(const BLEUuid &uuid, uint8_t properties, uint8_t permissions, uint8_t *value, int valueLength, int valueSize, bool fixedLength, bool constant) :
    m_uuid(uuid),
    m_properties(properties),
    m_permissions(permissions),
    m_constant(constant),
    m_fixed_length(fixedLength),
    m_value_data(value),
    m_value_size(valueSize),
    m_value_length(valueLength)
{
    m_refcount = 1;

    m_handle = 0;
    m_parent = nullptr;
    m_sibling = nullptr;

    m_control = 0;
    m_status = BLE_STATUS_SUCCESS;
    m_config = 0;

    m_write_length = 0;
    m_write_handle = 0x0000;
    
    m_request = nullptr;

    m_done_callback = Callback();
    m_read_callback = Callback();
    m_write_callback = Callback();
    m_subscribe_callback = Callback();
}

BLEServerCharacteristic::~BLEServerCharacteristic() {
    class BLEServerCharacteristic *characteristicCurrent, **characteristicPrevious;

    if (m_parent) {
        for (characteristicPrevious = &m_parent->m_children, characteristicCurrent = *characteristicPrevious; characteristicCurrent; characteristicPrevious = &characteristicCurrent->m_sibling, characteristicCurrent = *characteristicPrevious) {
            if (this == characteristicCurrent) {
                m_parent->m_count--;

                *characteristicPrevious = this->m_sibling;
            
                this->m_parent = nullptr;
                this->m_sibling = nullptr;
            }
        }
    }
            
    deallocate_noconst(const_cast<uint8_t*>(m_value_data));
}

void BLEServerCharacteristic::reference() {
    __armv7m_atomic_inch(&m_refcount, 0xffff);
}

void BLEServerCharacteristic::unreference() {
    if (__armv7m_atomic_dech(&m_refcount) == 1) {
        delete this;
    }
}

BLEServerCharacteristic::operator bool() const {
    return true;
}

class BLEServerCharacteristic* BLEServerCharacteristic::server() {
    return this;
}
    
const BLEUuid BLEServerCharacteristic::uuid() const {
    return m_uuid;
}

uint8_t BLEServerCharacteristic::properties() const {
    return m_properties;
}

bool BLEServerCharacteristic::fixedLength() const {
    return m_fixed_length;
}

int BLEServerCharacteristic::valueSize() {
    return m_value_size;
}

int BLEServerCharacteristic::getValue(void *value, int size) {
    int length;

    do {
        armv7m_atomic_orb(&m_control, BLE_SERVER_CONTROL_SEQUENCE);
      
        length = size;
        
        if (length > m_value_length) {
            length = m_value_length;
        }
        
        memcpy(value, &m_value_data[0], length);
    } while (!(m_control & BLE_SERVER_CONTROL_SEQUENCE));
    
    armv7m_atomic_andb(&m_control, ~(BLE_SERVER_CONTROL_MODIFIED | BLE_SERVER_CONTROL_WRITTEN | BLE_SERVER_CONTROL_SEQUENCE));
    
    return length;
}
  
bool BLEServerCharacteristic::setValue(const void *value, int length) {
    uint8_t control;
    
    if (m_constant) {
        return false;
    }
    
    if (m_fixed_length) {
        if (length != m_value_size) {
            return false;
        }
    } else {
        if (length > m_value_size) {
            return false;
        }
    }

    do {
        control = m_control;

        if (control & (BLE_SERVER_CONTROL_WRITTEN | BLE_SERVER_CONTROL_NOTIFY)) {
            return false;
        }
    } while(armv7m_atomic_casb(&m_control, control, (control | (BLE_SERVER_CONTROL_SET | BLE_SERVER_CONTROL_CHANGED))) != control);;

    do {
        armv7m_atomic_orb(&m_control, BLE_SERVER_CONTROL_SEQUENCE);
                            
        m_value_length = length;
    
        memcpy(&m_value_data[0], value, length);
    } while (!(m_control & BLE_SERVER_CONTROL_SEQUENCE));

    armv7m_atomic_andb(&m_control, ~BLE_SERVER_CONTROL_SEQUENCE);
    
    return true;
}

bool BLEServerCharacteristic::notifyValue(const void *value, int length, Callback callback) {
    BLEServerCharacteristic *request_submit;
    uint8_t control;

    if (!(m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE))) {
        return false;
    }

    if (m_constant) {
        return false;
    }
    
    if (m_fixed_length) {
        if (length != m_value_size) {
            return false;
        }
    } else {
        if (length > m_value_size) {
            return false;
        }
    }

    do {
        control = m_control;

        if (control & (BLE_SERVER_CONTROL_WRITTEN | BLE_SERVER_CONTROL_NOTIFY)) {
            return false;
        }
    } while(armv7m_atomic_casb(&m_control, control, ((control & ~(BLE_SERVER_CONTROL_SET | BLE_SERVER_CONTROL_CHANGED)) | BLE_SERVER_CONTROL_NOTIFY)) != control);

    m_value_length = length;
    
    memcpy(&m_value_data[0], value, length);

    m_status = BLE_STATUS_BUSY;
    
    m_done_callback = callback;
        
    do {
        request_submit = BLEHost.m_machine.request_submit;

        m_request = request_submit;
    } while ((BLEServerCharacteristic *)armv7m_atomic_cas((volatile uint32_t*)&BLEHost.m_machine.request_submit, (uint32_t)request_submit, (uint32_t)this) != request_submit);

    k_work_submit(&BLEHost.m_request_work);

    return true;
}

uint8_t BLEServerCharacteristic::status() {
  return m_status;
}

bool BLEServerCharacteristic::busy() {
    return (m_status == BLE_STATUS_BUSY);
}

bool BLEServerCharacteristic::written() {
    return !!(m_control & BLE_SERVER_CONTROL_WRITTEN);
}

bool BLEServerCharacteristic::subscribed() {
    return !!(m_config & ((m_properties & BLE_PROPERTY_INDICATE) ? 0xaaaa : 0x5555));
}
    
void BLEServerCharacteristic::onRead(Callback callback) {
    m_read_callback = callback;
}

void BLEServerCharacteristic::onWrite(Callback callback) {
    m_write_callback = callback;
}

void BLEServerCharacteristic::onSubscribe(Callback callback) {
    m_subscribe_callback = callback;
}

BLEServerService::BLEServerService(const BLEUuid &uuid) :
    m_uuid(uuid)
{
    m_refcount = 1;

    m_handle = 0;
    m_end_group_handle = 0;
    m_count = 0;
    m_parent = nullptr;
    m_sibling = nullptr;
    m_children = nullptr;
}

BLEServerService::~BLEServerService() {
    class BLEServerService *serviceCurrent, **servicePrevious;

    if (m_parent) {
        for (servicePrevious = &m_parent->m_server.children, serviceCurrent = *servicePrevious; serviceCurrent; servicePrevious = &serviceCurrent->m_sibling, serviceCurrent = *servicePrevious) {
            if (this == serviceCurrent) {
                m_parent->m_server.count--;

                *servicePrevious = this->m_sibling;
            
                this->m_parent = nullptr;
                this->m_sibling = nullptr;
            }
        }
    }
}

void BLEServerService::reference() {
    __armv7m_atomic_inch(&m_refcount, 0xffff);
}

void BLEServerService::unreference() {
    if (__armv7m_atomic_dech(&m_refcount) == 1) {
        delete this;
    }
}

BLEServerService::operator bool() const {
    return true;
}

class BLEServerService* BLEServerService::server() {
    return this;
}

const BLEUuid BLEServerService::uuid() const {
    return m_uuid;
}

unsigned int BLEServerService::characteristicCount() {
    return m_count;
}

BLECharacteristic BLEServerService::characteristic(unsigned int index) {
    class BLEServerCharacteristic *characteristicCurrent;

    for (characteristicCurrent = m_children; characteristicCurrent; characteristicCurrent = characteristicCurrent->m_sibling, index--) {
        if (index == 0) {
            return BLECharacteristic(characteristicCurrent);
        }
    }
    
    return BLECharacteristic();
}

BLECharacteristic BLEServerService::characteristic(const BLEUuid &uuid) {
    class BLEServerCharacteristic *characteristicCurrent;

    if (uuid) {
        for (characteristicCurrent = m_children; characteristicCurrent; characteristicCurrent = characteristicCurrent->m_sibling) {
            if (uuid == characteristicCurrent->m_uuid) {
                return BLECharacteristic(characteristicCurrent);
            }
        }
    }
    
    return BLECharacteristic();
}

bool BLEServerService::addCharacteristic(BLECharacteristic &characteristic) {
    class BLEServerCharacteristic *characteristicServer, *characteristicCurrent;

    characteristicServer = characteristic.interface()->server();

    if (!characteristicServer || characteristicServer->m_parent || BLEHost.m_server.active) {
        return false;
    }
    
    if (m_children) {
        for (characteristicCurrent = m_children; characteristicCurrent; characteristicCurrent = characteristicCurrent->m_sibling) {
            if (!characteristicCurrent->m_sibling) {
                characteristicCurrent->m_sibling = characteristicServer;
                break;
            }
        }
    } else {
        m_children = characteristicServer;
    }
    
    characteristicServer->reference();
                
    characteristicServer->m_parent = this;
                
    m_count++;
    
    return true;
}

/**********************************************************************************
 * GAP peer
 */

BLEPeerDevice::BLEPeerDevice() {
    m_refcount = 1;

    m_index = 0;
    m_handle = BLE_PEER_DEVICE_NOT_CONNECTED;
    
    m_address = BLEAddress();
    m_mtu = BLE_MIN_ATT_MTU;

    m_connection.tx_phy = BLE_PHY_1M;
    m_connection.rx_phy = BLE_PHY_1M;
    m_connection.interval = 0;
    m_connection.latency = 0;
    m_connection.timeout = 0;

    m_client.discovered = false;
    m_client.count = 0;
    m_client.children = nullptr;

    m_machine.connected = false;

    m_machine.request = 0;
    m_machine.event = 0;

    m_machine.connect_request = false;
    m_machine.disconnect_request = false;
    m_machine.params_request = false;
    
    m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;
    m_machine.discovery_service = nullptr;
    m_machine.discovery_characteristic = nullptr;

    m_machine.request_submit = nullptr;
    m_machine.request_head = nullptr;
    m_machine.request_tail = nullptr;

    m_machine.read_current = nullptr;
    m_machine.write_current = nullptr;

    m_machine.config_request = false;
    m_machine.config_current = nullptr;

    m_machine.modified_request = false;
    m_machine.confirm_request = false;
    
    Callback request_callback = Callback(&BLEPeerDevice::request, this);

    k_work_init(&m_request_work, request_callback.callback(), request_callback.context());

    m_done_callback = Callback();
    m_mtu_exchange_callback = Callback();
    m_connection_parameters_callback = Callback();
    m_disconnect_callback = Callback();
}

BLEPeerDevice::~BLEPeerDevice() {
}

BLEPeerDevice::operator bool() const {
    return true;
}

void BLEPeerDevice::reference() {
    __armv7m_atomic_inch(&m_refcount, 0xffff);
}

void BLEPeerDevice::unreference() {
    if (__armv7m_atomic_dech(&m_refcount) == 1) {
        delete this;
    }
}

const BLEAddress BLEPeerDevice::address() const {
    return m_address;
}

int BLEPeerDevice::mtu() {
    return m_mtu;
}

void BLEPeerDevice::disconnect() {
    m_machine.disconnect_request = true;
    
    k_work_submit(&m_request_work);
}

bool BLEPeerDevice::connected() {
    return (m_handle != BLE_PEER_DEVICE_NOT_CONNECTED);
}

bool BLEPeerDevice::discover(Callback callback) {
    if (armv7m_atomic_casb(&m_machine.discovery_request, BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE, BLE_PEER_DEVICE_DISCOVERY_REQUEST_NOT_READY) != BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE) {
        return false;
    }

    m_done_callback = callback;
    m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER;
    
    k_work_submit(&m_request_work);
    
    return true;
}

bool BLEPeerDevice::discoverServices(Callback callback) {
    if (armv7m_atomic_casb(&m_machine.discovery_request, BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE, BLE_PEER_DEVICE_DISCOVERY_REQUEST_NOT_READY) != BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE) {
        return false;
    }
    
    m_done_callback = callback;
    m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_SERVICES;
    
    k_work_submit(&m_request_work);
    
    return true;

}

bool BLEPeerDevice::discoverCharacteristics(const BLEUuid &uuid, Callback callback) {
    if (armv7m_atomic_casb(&m_machine.discovery_request, BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE, BLE_PEER_DEVICE_DISCOVERY_REQUEST_NOT_READY) != BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE) {
        return false;
    }

    m_done_callback = callback;
    m_machine.discovery_uuid = uuid;
    m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_UUID;
    
    k_work_submit(&m_request_work);
    
    return true;
}

bool BLEPeerDevice::discoverCharacteristics(BLEService service, Callback callback) {
    BLEClientService *client_service;

    client_service = service.interface()->client();

    if (!client_service) {
        return false;
    }

    if (armv7m_atomic_casb(&m_machine.discovery_request, BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE, BLE_PEER_DEVICE_DISCOVERY_REQUEST_NOT_READY) != BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE) {
        return false;
    }

    m_done_callback = callback;
    m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_SERVICE;
    
    k_work_submit(&m_request_work);
    
    return true;
}

unsigned int BLEPeerDevice::serviceCount() {
    return m_client.count;
}

BLEService BLEPeerDevice::service(unsigned int index) {
    class BLEClientService *serviceCurrent;

    for (serviceCurrent = m_client.children; serviceCurrent; serviceCurrent = serviceCurrent->m_sibling, index--) {
        if (index == 0) {
            return BLEService(serviceCurrent);
        }
    }

    return BLEService();
}

BLEService BLEPeerDevice::service(const BLEUuid &uuid) {
    class BLEClientService *serviceCurrent;

    if (uuid) {
        for (serviceCurrent = m_client.children; serviceCurrent; serviceCurrent = serviceCurrent->m_sibling) {
            if (uuid == serviceCurrent->m_uuid) {
                return BLEService(serviceCurrent);
            }
        }
    }
    
    return BLEService();
}

void BLEPeerDevice::getPhys(BLEPhy &tx_phy, BLEPhy &rx_phy) {
    tx_phy = (BLEPhy)m_connection.tx_phy;
    rx_phy = (BLEPhy)m_connection.rx_phy;
}

bool BLEPeerDevice::setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout) {
    if (interval_min > interval_max) {
        return false;
    }
    
    if ((interval_min < 0x0006) || (interval_min > 0x0c80)) {
        return false;
    }

    if ((interval_max < 0x0006) || (interval_max > 0x0c80)) {
        return false;
    }

    if (latency > 0x01f3) {
        return false;
    }
    
    if ((timeout < 0x000a) || (timeout > 0x0c80)) {
        return false;
    }

    m_machine.params.interval_min = interval_min;
    m_machine.params.interval_max = interval_max;
    m_machine.params.latency = latency;
    m_machine.params.timeout = timeout;

    m_machine.params_request = true;

    k_work_submit(&m_request_work);
    
    return true;
}

void BLEPeerDevice::getConnectionParameters(uint16_t &connectionInterval, uint16_t &latency, uint16_t &timeout) {
    connectionInterval = m_connection.interval;
    latency = m_connection.latency;
    timeout = m_connection.timeout;
}

void BLEPeerDevice::onMTUExchange(Callback callback) {
    m_mtu_exchange_callback = callback;
}

void BLEPeerDevice::onConnectionParameters(Callback callback) {
    m_connection_parameters_callback = callback;
}

void BLEPeerDevice::onDisconnect(Callback callback) {
    m_disconnect_callback = callback;
}

void BLEPeerDevice::request() {
    BLEClientService *service;
    BLEClientCharacteristic *characteristic;
    BLEClientCharacteristic *request_submit, *request_next, *request_head, *request_tail;
    uint8_t control;

    if (!m_machine.connected) {
        if (m_machine.connect_request) {
            m_machine.connect_request = false;
        
            armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GAP_CREATE_CONNECTION);
        }
    } else {
        if (m_machine.disconnect_request) {
            m_machine.disconnect_request = false;
        
            armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GAP_TERMINATE);
        }
    
        if (m_machine.params_request) {
            m_machine.params_request = false;
        
            armv7m_atomic_or(&m_machine.request, BLE_REQUEST_L2CAP_CONNECTION_PARAMETER_UPDATE);
        }

        if ((m_machine.discovery_request > BLE_PEER_DEVICE_DISCOVERY_REQUEST_NOT_READY) && !(m_machine.request & (BLE_REQUEST_GATT_DISCOVER_SERVICES | BLE_REQUEST_GATT_DISCOVER_CHARACTERISTICS | BLE_REQUEST_GATT_DISCOVER_DESCRIPTORS))) {
            switch (m_machine.discovery_request) {
            case BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE:
            case BLE_PEER_DEVICE_DISCOVERY_REQUEST_NOT_READY:
                break;

            case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER:
                if (m_client.discovered) {
                    for (service = m_client.children; service; service = service->m_sibling) {
                        if (((service->m_handle +1) != service->m_end_group_handle) && !service->m_children) {
                            break;
                        }
                    }

                    if (!service) {
                        m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;

                        m_done_callback();
                    } else {
                        m_machine.discovery_service = service;
                        m_machine.discovery_characteristic = nullptr;

                        armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_DISCOVER_CHARACTERISTICS);
                    }
                } else {
                    m_machine.discovery_service = nullptr;
                    m_machine.discovery_characteristic = nullptr;
            
                    armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_DISCOVER_SERVICES);
                }
                break;
        
            case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_SERVICES:
                m_machine.discovery_service = nullptr;
                m_machine.discovery_characteristic = nullptr;

                armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_DISCOVER_SERVICES);
                break;

            case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_UUID:
                m_machine.discovery_service = nullptr;
                m_machine.discovery_characteristic = nullptr;

                for (service = m_client.children; service; service = service->m_sibling) {
                    if (service->m_uuid == m_machine.discovery_uuid) {
                        m_machine.discovery_service = service;
                        break;
                    }
                }

                if (m_machine.discovery_service) {
                    service = m_machine.discovery_service;
                    
                    if (((service->m_handle +1) != service->m_end_group_handle) && !service->m_children) {
                        m_machine.discovery_characteristic = nullptr;
                        
                        armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_DISCOVER_CHARACTERISTICS);
                    } else {
                        m_machine.procedure = BLE_GATT_PROCEDURE_NONE;
                        m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;
                        
                        m_done_callback();
                    }
                } else {
                    armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_DISCOVER_SERVICE_BY_UUID);
                }
                break;
            
            case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_SERVICE:
                m_machine.discovery_characteristic = nullptr;

                armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_DISCOVER_CHARACTERISTICS);
                break;
            }
        }

        if (m_machine.request_submit) {
            request_submit = (class BLEClientCharacteristic *)armv7m_atomic_swap((volatile uint32_t*)&m_machine.request_submit, (uint32_t)nullptr);

            for (request_head = nullptr, request_tail = request_submit; request_submit != nullptr; request_submit = request_next)
            {
                request_next = request_submit->m_request;

                request_submit->m_request = request_head;

                request_head = request_submit;
            }
        
            if (!m_machine.request_head)
            {
                m_machine.request_head = request_head;
            }
            else
            {
                m_machine.request_tail->m_request = request_head;
            }
        
            m_machine.request_tail = request_tail;
        }

        if (m_machine.request_head && !m_machine.read_current && !m_machine.write_current) {
            characteristic = m_machine.request_head;
            m_machine.request_head = characteristic->m_request;
            if (m_machine.request_tail == characteristic) {
                m_machine.request_tail = nullptr;
            }
            characteristic->m_request = nullptr;

            if (characteristic->m_control & BLE_CLIENT_CONTROL_READ) {
                m_machine.read_current = characteristic;
                m_machine.read_offset = 0;

                armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_READ_VALUE);
            }

            if (characteristic->m_control & BLE_CLIENT_CONTROL_WRITE) {
                m_machine.write_current = characteristic;

                if (characteristic->m_control & BLE_CLIENT_CONTROL_WRITE_WITHOUT_RESPONSE) {
                    armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_WRITE_VALUE_WITHOUT_RESPONSE);
                } else {
                    if (characteristic->m_value_length <= (m_mtu -3)) {
                        armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_WRITE_VALUE);
                    } else {
                        armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_WRITE_VALUE_LONG);
                    }
                }
            }
        }
    
        if (m_machine.config_request && !m_machine.config_current) {
            m_machine.config_request = false;

            for (service = m_client.children; service; service = service->m_sibling) {
                for (characteristic = service->m_children; characteristic; characteristic = characteristic->m_sibling) {
                    if (characteristic->m_control & BLE_CLIENT_CONTROL_CONFIG) {
                        m_machine.config_request = true;
                        m_machine.config_current = characteristic;

                        armv7m_atomic_andb(&characteristic->m_control, ~BLE_CLIENT_CONTROL_CONFIG);
                    
                        armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_WRITE_CONFIG);
                        break;
                    }
                }
                
                if (m_machine.config_current) {
                    break;
                }
            }
        }

        if (m_machine.modified_request) {
            m_machine.modified_request = false;

            for (service = m_client.children; service; service = service->m_sibling) {
                for (characteristic = service->m_children; characteristic; characteristic = characteristic->m_sibling) {
                    if ((characteristic->m_control & (BLE_CLIENT_CONTROL_MODIFIED | BLE_CLIENT_CONTROL_WRITTEN)) == BLE_CLIENT_CONTROL_MODIFIED) {
                        do {
                            control = characteristic->m_control;
                        
                            if (control & (BLE_CLIENT_CONTROL_READ | BLE_CLIENT_CONTROL_WRITE)) {
                                break;
                            }
                        } while (armv7m_atomic_casb(&characteristic->m_control, control, (control | BLE_CLIENT_CONTROL_WRITTEN)) != control);
                    
                        if (!(control & (BLE_CLIENT_CONTROL_READ | BLE_CLIENT_CONTROL_WRITE))) {
#if (BLE_TRACE_SUPPORTED == 1)
                            armv7m_rtt_printf("ATTRIB-NOTIFY (\"%s\")\r\n", uuid_string(characteristic->m_uuid));
#endif

                            characteristic->m_value_length = 0;
                        
                            if (characteristic->m_value_size < characteristic->m_write_length) {
                                characteristic->m_value_data = (uint8_t*)realloc(characteristic->m_value_data, characteristic->m_write_length);
                            
                                if (characteristic->m_value_data) {
                                    characteristic->m_value_size = characteristic->m_write_length;
                                } else {
                                    characteristic->m_value_size = 0;
                                }
                            }
                        
                            if (characteristic->m_value_size) {
                                memcpy(&characteristic->m_value_data[0], &characteristic->m_write_data[0], characteristic->m_write_length);
                            
                                characteristic->m_value_length = characteristic->m_write_length;
                            }
                        
                            characteristic->m_notify_callback();
                        }
                    }
                }
            }
        }
    
        if (m_machine.confirm_request) {
            m_machine.confirm_request = false;

            armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_CONFIRM_INDICATION);
        }
    }
    
    BLEHost.process();
}

/**********************************************************************************
 * GAP host
 */

int hci_send_req(stm32wb_ipcc_ble_command_t *command, bool async) {
    if (async) {
        return BLE_STATUS_ERROR;
    }

    command->next = NULL;
    command->rsize = command->rlen;
    command->callback = (stm32wb_ipcc_ble_command_callback_t)done_callback;
    command->context = k_task_self();

    if (!stm32wb_ipcc_ble_command(command)) {
        return BLE_STATUS_ERROR;
    }

    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

    return (command->status == STM32WB_IPCC_BLE_COMMAND_STATUS_SUCCESS) ? BLE_STATUS_SUCCESS : BLE_STATUS_TIMEOUT;
}

BLELocalDevice::BLELocalDevice() {
    armv7m_sha256_context_t sha256_ctx;
    uint32_t hash[8];
    
    m_state = BLE_LOCAL_STATE_NONE;

    m_address = BLEAddress();

    armv7m_sha256_init(&sha256_ctx);
    armv7m_sha256_update(&sha256_ctx, (const uint8_t*)0x1fff7400, 1024); /* ENG BYTES */
    armv7m_sha256_final(&sha256_ctx, &hash[0]);

    memcpy(&m_er[0], &hash[0], 16);
    memcpy(&m_ir[0], &hash[4], 16);
    
    m_connection.pa_level = 0x19;
    m_connection.tx_power_level = 0;
    m_connection.all_phys = 3;
    m_connection.tx_phy = BLE_PHY_1M;
    m_connection.rx_phy = BLE_PHY_1M;
    m_connection.phy_options = 0;
    m_connection.interval_min = BLE_GAP_DEFAULT_INTERVAL_MIN;
    m_connection.interval_max = BLE_GAP_DEFAULT_INTERVAL_MAX;
    m_connection.latency = BLE_GAP_DEFAULT_LATENCY;
    m_connection.timeout = BLE_GAP_DEFAULT_TIMEOUT;
    m_connection.min_ce_length = BLE_GAP_DEFAULT_MIN_CE_LENGTH;
    m_connection.max_ce_length = BLE_GAP_DEFAULT_MAX_CE_LENGTH;

    m_gap.device_name = nullptr;
    m_gap.device_name_length = BLE_MIN_DEVICE_NAME_LENGTH;
    m_gap.appearance = 0;
    m_gap.bonding = false;
    m_gap.security = BLE_SECURITY_UNENCRYPTED;
    m_gap.passkey = BLE_PASSKEY_UNDEFINED;

    m_peripheral.connectable = true;
    m_peripheral.discoverable = BLE_DISCOVERABLE_GENERAL;
    m_peripheral.advertising_interval_min = 0x0080; /*  80ms */
    m_peripheral.advertising_interval_max = 0x00a0; /* 100ms */
    m_peripheral.advertising_data = BLEAdvertisingData(BLE_AD_FLAGS_BR_EDR_NOT_SUPPORTED);
    m_peripheral.scan_response_data = BLEAdvertisingData();

    m_central.scan_type = BLE_SCAN_TYPE_ACTIVE;
    m_central.scan_interval = 0x00a0; /* 100ms */
    m_central.scan_window = 0x0050; /*  50ms */
    
    m_server.active = false;
    m_server.count = 0;
    m_server.children = nullptr;

    m_machine.busy = BLE_BUSY_NONE;
    m_machine.request = 0;
    m_machine.event = 0;
    m_machine.index = BLE_NUM_LINK -1;
    m_machine.device = nullptr;
    
    // m_machine.central = BLEPeerDevice();
    m_machine.peer[0] = nullptr;
    m_machine.peer[1] = nullptr;
    m_machine.peer[2] = nullptr;
    m_machine.peer[3] = nullptr;
    
    m_machine.terminate_request = 0;

    m_machine.request_submit = nullptr;
    m_machine.request_head = nullptr;
    m_machine.request_tail = nullptr;

    m_machine.notify_current = nullptr;
    m_machine.set_request = false;

    m_machine.report_index = 0;
    m_machine.report_read = 0;
    m_machine.report_write = 0;
    
    m_machine.command.next = NULL;
    m_machine.command.opcode = 0;
    m_machine.command.event = 0;
    m_machine.command.cparam = &m_machine.data[0];
    m_machine.command.rparam = &m_machine.data[0];
    m_machine.command.rsize = 255;
    m_machine.command.status = STM32WB_IPCC_BLE_COMMAND_STATUS_SUCCESS;
    m_machine.command.callback = (stm32wb_ipcc_ble_command_callback_t)k_work_submit;
    m_machine.command.context = (void*)&m_done_work;

    Callback done_callback = Callback(&BLELocalDevice::done, this);

    k_work_init(&m_done_work, done_callback.callback(), done_callback.context());

    Callback event_callback = Callback(&BLELocalDevice::event, this);

    k_work_init(&m_event_work, event_callback.callback(), event_callback.context());
    
    Callback request_callback = Callback(&BLELocalDevice::request, this);

    k_work_init(&m_request_work, request_callback.callback(), request_callback.context());

    m_stop_callback = Callback();
    m_report_callback = Callback();
    m_connect_callback = Callback();
    m_disconnect_callback = Callback();
}

bool BLELocalDevice::begin(uint32_t options, int mtu) {
    tBleStatus status = BLE_STATUS_SUCCESS;
    uint16_t maxTxOctets, maxTxTime, maxRxOctets, maxRxTime;

    if (!(options & (BLE_OPTION_ROLE_PERIPHERAL | BLE_OPTION_ROLE_BROADCASTER | BLE_OPTION_ROLE_CENTRAL | BLE_OPTION_ROLE_OBSERVER))) {
        options |= BLE_OPTION_ROLE_PERIPHERAL;
    }

    if (options & BLE_OPTION_ROLE_PERIPHERAL) {
        options |= BLE_OPTION_ROLE_BROADCASTER;
    }

    if (options & BLE_OPTION_ROLE_CENTRAL) {
        options |= BLE_OPTION_ROLE_OBSERVER;
    }
    
    if ((mtu < BLE_MIN_ATT_MTU) || (mtu > BLE_MAX_ATT_MTU)) {
        return false;
    }
    
    if (m_state != BLE_LOCAL_STATE_NONE) {
        return false;
    }

    m_state = BLE_LOCAL_STATE_NOT_READY;
    m_mtu = mtu;
    m_options = options;

    m_public_address[0] = *((const uint8_t*)(UID64_BASE + 0));
    m_public_address[1] = *((const uint8_t*)(UID64_BASE + 1));
    m_public_address[2] = *((const uint8_t*)(UID64_BASE + 2));
    m_public_address[3] = 0x26;
    m_public_address[4] = 0xe1;
    m_public_address[5] = 0x80;

    if (!(m_random_static_address[5] & 0xc0)) {
        volatile uint8_t busy;

        stm32wb_random(&m_random_static_address[0], sizeof(m_random_static_address), &busy, NULL, NULL);
        
        while (busy == STM32WB_RANDOM_STATUS_BUSY) {
            __WFE();
        }
            
        m_random_static_address[5] |= 0xc0;
    }

    stm32wb_ipcc_ble_init_params_t init_params = {
        NULL,                                                                                          /* pBleBufferAddress          */
        0,                                                                                             /* BleBufferSize              */
        BLE_NUM_GATT_ATTRIBUTES,                                                                       /* NumAttrRecord              */
        BLE_NUM_GATT_SERVICES,                                                                         /* NumAttrServ                */
        BLE_ATT_VALUE_ARRAY_SIZE,                                                                      /* AttrValueArrSize           */
        BLE_NUM_LINK,                                                                                  /* NumOfLinks                 */
        BLE_DATA_LENGTH_EXTENSION,                                                                     /* ExtendedPacketLengthEnable */
        BLE_PREP_WRITE_X_ATT(BLE_MAX_ATT_SIZE),                                                        /* PrWriteListSize            */
        (uint8_t)BLE_MBLOCKS_CALC(BLE_PREP_WRITE_X_ATT(BLE_MAX_ATT_SIZE), mtu, BLE_NUM_LINK),          /* MblockCount                */
        (uint8_t)mtu,                                                                                  /* AttMtu                     */
        BLE_SLAVE_SCA,                                                                                 /* SlaveSca                   */
        BLE_MASTER_SCA,                                                                                /* MasterSca                  */
        BLE_LSE_SOURCE,                                                                                /* LsSource                   */
        BLE_MAX_CONN_EVENT_LENGTH,                                                                     /* MaxConnEventLength         */
        BLE_HSE_STARTUP_TIME,                                                                          /* HsStartupTime              */
        BLE_VITERBI_MODE,                                                                              /* ViterbiEnable              */
        (uint8_t)(BLE_OPTION_REDUCED_GATT_DATABASE_IN_NVM | BLE_OPTION_READ_ONLY_DEVICE_NAME),         /* Options                    */
        0,                                                                                             /* HwVersion                  */
        32,                                                                                            /* MaxCOCInitiatorNbr         */
        BLE_MIN_TX_POWER,                                                                              /* MinTxPower                 */
        BLE_MAX_TX_POWER,                                                                              /* MaxTxPower                 */
        BLE_RX_MODEL,                                                                                  /* RxModelConfig              */
        0,                                                                                             /* MaxAdvSetNbr               */
        0,                                                                                             /* MaxAdvDataLen              */
        BLE_TX_PATH_COMPENSATION,                                                                      /* TxPathCompensation         */
        BLE_RX_PATH_COMPENSATION,                                                                      /* RxPacthCompensation        */
        BLE_CORE_VERSION_5_2,                                                                          /* BleCoreVersion             */
        0                                                                                              /* OptionsExt                 */
    };

    if (!stm32wb_ipcc_sys_enable()) {
        return false;
    }

    if (!stm32wb_ipcc_ble_enable(&init_params, (stm32wb_ipcc_ble_event_callback_t)k_work_submit, (void*)&m_event_work)) {
        armv7m_rtt_printf("FAILURE\r\n");
      
        stm32wb_ipcc_sys_disable();
        
        return false;
    }
    
    status = hci_reset();
    
    if (status == BLE_STATUS_SUCCESS) {
        if (!(options & BLE_OPTION_RANDOM_STATIC_ADDRESS)) {
            m_address = BLEAddress(m_public_address, BLE_ADDRESS_TYPE_PUBLIC);
        }
        
        status = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, m_public_address);
    }

    if (status == BLE_STATUS_SUCCESS) {
        status = aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, &m_er[0]);
    }
        
    if (status == BLE_STATUS_SUCCESS) {
        status = aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, &m_ir[0]);
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        if (options & BLE_OPTION_RANDOM_STATIC_ADDRESS) {
            m_address = BLEAddress(m_random_static_address, BLE_ADDRESS_TYPE_RANDOM_STATIC);
        }
        
        status = aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, m_random_static_address);
    }

    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gatt_init();

#if 1        
        if (status == BLE_STATUS_SUCCESS) {
            /*
             *        - 0x00000001: ACI_GATT_ATTRIBUTE_MODIFIED_EVENT
             *        - 0x00000002: ACI_GATT_PROC_TIMEOUT_EVENT
             *        - 0x00000004: ACI_ATT_EXCHANGE_MTU_RESP_EVENT
             *        - 0x00000008: ACI_ATT_FIND_INFO_RESP_EVENT
             *        - 0x00000010: ACI_ATT_FIND_BY_TYPE_VALUE_RESP_EVENT
             *        - 0x00000020: ACI_ATT_READ_BY_TYPE_RESP_EVENT
             *        - 0x00000040: ACI_ATT_READ_RESP_EVENT
             *        - 0x00000080: ACI_ATT_READ_BLOB_RESP_EVENT
             *        - 0x00000100: ACI_ATT_READ_MULTIPLE_RESP_EVENT
             *        - 0x00000200: ACI_ATT_READ_BY_GROUP_TYPE_RESP_EVENT
             *        - 0x00000800: ACI_ATT_PREPARE_WRITE_RESP_EVENT
             *        - 0x00001000: ACI_ATT_EXEC_WRITE_RESP_EVENT
             *        - 0x00002000: ACI_GATT_INDICATION_EVENT
             *        - 0x00004000: ACI_GATT_NOTIFICATION_EVENT
             *        - 0x00008000: ACI_GATT_ERROR_RESP_EVENT
             *        - 0x00010000: ACI_GATT_PROC_COMPLETE_EVENT
             *        - 0x00020000: ACI_GATT_DISC_READ_CHAR_BY_UUID_RESP_EVENT
             *        - 0x00040000: ACI_GATT_TX_POOL_AVAILABLE_EVENT
             *        - 0x00100000: ACI_GATT_READ_EXT_EVENT
             *        - 0x00200000: ACI_GATT_INDICATION_EXT_EVENT
             *        - 0x00400000: ACI_GATT_NOTIFICATION_EXT_EVENT
             */

          status =  aci_gatt_set_event_mask(0x0007e3ff);
          // status =  aci_gatt_set_event_mask(0xffffffff);
        }
#endif        
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        if (m_gap.device_name) {
            m_gap.device_name_length = strlen(m_gap.device_name);
            
            if (m_gap.device_name_length > BLE_MAX_DEVICE_NAME_LENGTH) {
                m_gap.device_name_length = BLE_MAX_DEVICE_NAME_LENGTH;
            }
        }
        
        status = aci_gap_init(GAP_PERIPHERAL_ROLE | GAP_BROADCASTER_ROLE | GAP_CENTRAL_ROLE | GAP_OBSERVER_ROLE,
                              ((m_options & BLE_OPTION_PRIVACY) ? PRIVACY_ENABLED : PRIVACY_DISABLED),
                              m_gap.device_name_length,
                              &m_gap.service_handle,
                              &m_gap.device_name_handle,
                              &m_gap.appearance_handle);
        
        if (status == BLE_STATUS_SUCCESS) {
            m_gap.connection_parameter_handle = m_gap.appearance_handle + 2;
        }
    }

    if (status == BLE_STATUS_SUCCESS) {
        status = authentication();
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gatt_update_char_value(m_gap.service_handle,
                                            m_gap.device_name_handle,
                                            0,
                                            (m_gap.device_name ? strlen(m_gap.device_name) : 0),
                                            reinterpret_cast<const uint8_t*>(m_gap.device_name));
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gatt_update_char_value(m_gap.service_handle,
                                            m_gap.appearance_handle,
                                            0,
                                            2,
                                            reinterpret_cast<const uint8_t*>(&m_gap.appearance));
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gatt_update_char_value(m_gap.service_handle,
                                            m_gap.connection_parameter_handle,
                                            0,
                                            8,
                                            reinterpret_cast<const uint8_t*>(&m_connection.interval_min));
    }

    if (status == BLE_STATUS_SUCCESS) {
        status = hci_le_set_default_phy(m_connection.all_phys, m_connection.tx_phy, m_connection.rx_phy);
    }


    if (status == BLE_STATUS_SUCCESS) {
        status = hci_le_read_maximum_data_length(&maxTxOctets, &maxTxTime, &maxRxOctets, &maxRxTime);
    }

    if (status == BLE_STATUS_SUCCESS) {
        if (mtu < (maxTxOctets - 4)) {
            maxTxOctets = mtu + 4;
            maxTxTime = (maxTxOctets + 14) * 8; 
        }

#if (BLE_TRACE_SUPPORTED == 1)
        armv7m_rtt_printf("SUGGESTED-DEFAULT-DATA-LENGTH %d/%d /%d/%d\r\n", maxTxOctets, maxTxTime, maxRxOctets, maxRxTime);
#endif
        
        status = hci_le_write_suggested_default_data_length(maxTxOctets, maxTxTime);
    }

    
    if (status == BLE_STATUS_SUCCESS) {
        stm32wb_system_smps_configure(m_connection.pa_level);

        status = aci_hal_set_tx_power_level(0, m_connection.pa_level);
    }

    if (status != BLE_STATUS_SUCCESS) {
        aci_hal_stack_reset();

        stm32wb_ipcc_ble_disable();
        stm32wb_ipcc_sys_disable();

        m_state = BLE_LOCAL_STATE_NONE;
        
        return false;
    }
    
    m_state = BLE_LOCAL_STATE_READY;
    
    return true;
}

void BLELocalDevice::end() {
    BLEPeerDevice *device;
    int index;
    k_task_t *task_self = k_task_self();

    if (!task_self) {
        return;
    }
    
    if (m_state == BLE_LOCAL_STATE_NONE) {
        return;
    }
    
    m_state = BLE_LOCAL_STATE_NONE;

    m_machine.terminate_waiting = task_self;

    do {
        device = nullptr;
        
        for (index = 0; index < BLE_NUM_LINK; index++) {
            if (m_machine.peer[index]) {
                device = m_machine.peer[index];
                break;
            }
        }

        if (device) {
            device->m_machine.disconnect_request = true;
    
            k_work_submit(&device->m_request_work);

            k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);
        }
    } while (device);
    
    teardown();
        
    m_machine.busy = 0;
    m_machine.request = 0;
    m_machine.event = 0;

    aci_hal_stack_reset();

    stm32wb_ipcc_ble_disable();
    stm32wb_ipcc_sys_disable();
}

const BLEAddress BLELocalDevice::address() const {
    return m_address;
}

bool BLELocalDevice::setTxPowerLevel(int txPower) {
    uint8_t pa_level, tx_power_level; 

    if      (txPower >=   6) { pa_level = 0x1f; tx_power_level =   6; }
    else if (txPower >=   5) { pa_level = 0x1e; tx_power_level =   5; }
    else if (txPower >=   4) { pa_level = 0x1d; tx_power_level =   4; }
    else if (txPower >=   3) { pa_level = 0x1c; tx_power_level =   3; }
    else if (txPower >=   2) { pa_level = 0x1b; tx_power_level =   2; }
    else if (txPower >=   1) { pa_level = 0x1a; tx_power_level =   1; }
    else if (txPower >=   0) { pa_level = 0x19; tx_power_level =   0; }
    else if (txPower >=  -1) { pa_level = 0x16; tx_power_level =  -1; }
    else if (txPower >=  -2) { pa_level = 0x14; tx_power_level =  -2; }
    else if (txPower >=  -3) { pa_level = 0x12; tx_power_level =  -3; }
    else if (txPower >=  -4) { pa_level = 0x11; tx_power_level =  -4; }
    else if (txPower >=  -5) { pa_level = 0x10; tx_power_level =  -5; }
    else if (txPower >=  -6) { pa_level = 0x0f; tx_power_level =  -6; }
    else if (txPower >=  -7) { pa_level = 0x0e; tx_power_level =  -7; }
    else if (txPower >=  -8) { pa_level = 0x0d; tx_power_level =  -8; }
    else if (txPower >=  -9) { pa_level = 0x0c; tx_power_level =  -9; }
    else if (txPower >= -10) { pa_level = 0x0b; tx_power_level = -10; }
    else if (txPower >= -11) { pa_level = 0x0a; tx_power_level = -11; }
    else if (txPower >= -12) { pa_level = 0x09; tx_power_level = -12; }
    else if (txPower >= -13) { pa_level = 0x08; tx_power_level = -13; }
    else if (txPower >= -14) { pa_level = 0x07; tx_power_level = -14; }
    else if (txPower >= -15) { pa_level = 0x06; tx_power_level = -15; }
    else if (txPower >= -16) { pa_level = 0x05; tx_power_level = -16; }
    else if (txPower >= -17) { pa_level = 0x04; tx_power_level = -17; }
    else if (txPower >= -18) { pa_level = 0x03; tx_power_level = -18; }
    else if (txPower >= -19) { pa_level = 0x02; tx_power_level = -19; }
    else if (txPower >= -20) { pa_level = 0x01; tx_power_level = -20; }
    else                     { pa_level = 0x00; tx_power_level = -40; }

    m_connection.pa_level = pa_level;
    m_connection.tx_power_level = tx_power_level;
    
    if (m_state >= BLE_LOCAL_STATE_READY) {
        if (pa_level > m_connection.pa_level) {
            stm32wb_system_smps_configure(pa_level);
        }
        
        aci_hal_set_tx_power_level(0, pa_level);
        
        if (pa_level < m_connection.pa_level) {
            stm32wb_system_smps_configure(pa_level);
        }

        if (m_state == BLE_LOCAL_STATE_ADVERTISING) {
            if (m_machine.advertising_data.setTxPowerLevel(m_connection.tx_power_level)) {
                hci_le_set_advertising_data(m_machine.advertising_data.length(), m_machine.advertising_data.data());
            }
        }
    }        
    
    return true;
}

bool BLELocalDevice::setPreferredPhy(BLEPhy txPhy, BLEPhy rxPhy, BLEPhyOption phyOptions) {
    if ((txPhy & ~(BLE_PHY_1M | BLE_PHY_2M | BLE_PHY_CODED)) || (rxPhy & ~(BLE_PHY_1M | BLE_PHY_2M | BLE_PHY_CODED)) || (phyOptions & ~(BLE_PHY_OPTION_S2 | BLE_PHY_OPTION_S8))) {
        return false;
    }
    
    if ((txPhy & BLE_PHY_CODED) || (rxPhy & BLE_PHY_CODED) || phyOptions) {
        return false;
    }

    if (txPhy) {
        m_connection.all_phys &= ~1;

        m_connection.tx_phy = txPhy;
    } else {
        m_connection.all_phys |= 1;

        m_connection.tx_phy = 0;
    }

    if (rxPhy) {
        m_connection.all_phys &= ~2;

        m_connection.rx_phy = rxPhy;
    } else {
        m_connection.all_phys |= 2;

        m_connection.rx_phy = 0;
    }

    m_connection.phy_options = phyOptions;

    if (m_state >= BLE_LOCAL_STATE_READY) {
        hci_le_set_default_phy(m_connection.all_phys, m_connection.tx_phy, m_connection.rx_phy);
    }
    
    return true;
}

bool BLELocalDevice::setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout) {
    if (interval_min > interval_max) {
        return false;
    }
    
    if ((interval_min < 0x0006) || (interval_min > 0x0c80)) {
        return false;
    }

    if ((interval_max < 0x0006) || (interval_max > 0x0c80)) {
        return false;
    }

    if (latency > 0x01f3) {
        return false;
    }
    
    if ((timeout < 0x000a) || (timeout > 0x0c80)) {
        return false;
    }

    m_connection.interval_min = interval_min;
    m_connection.interval_max = interval_max;
    m_connection.latency = latency;
    m_connection.timeout = timeout;

    if (m_state >= BLE_LOCAL_STATE_READY) {
        aci_gatt_update_char_value(m_gap.service_handle,
                                   m_gap.connection_parameter_handle,
                                   0,
                                   8,
                                   reinterpret_cast<const uint8_t*>(&m_connection.interval_min));

        
        if (m_state == BLE_LOCAL_STATE_ADVERTISING) {
            if (m_machine.advertising_data.setConnectionInterval(m_connection.interval_min, m_connection.interval_max)) {
                hci_le_set_advertising_data(m_machine.advertising_data.length(), m_machine.advertising_data.data());
            }
        }
    }

    return true;
}

bool BLELocalDevice::setDeviceName(const char *deviceName) {
    uint32_t length = 0;

    if (deviceName) {
        length = strlen(deviceName);

        if (length > BLE_MAX_DEVICE_NAME_LENGTH) {
            return false;
        }
    }
    
    if (m_gap.device_name) {
        deallocate_noconst(m_gap.device_name);

        m_gap.device_name = NULL;
    }

    if (deviceName) {
        m_gap.device_name = (const char*)allocate_noconst(deviceName, length +1);
        m_gap.device_name_length = length;
    } else {
        m_gap.device_name = nullptr;
        m_gap.device_name_length = 0;
    }
    
    if (m_state == BLE_LOCAL_STATE_READY) {
        aci_gatt_update_char_value(m_gap.service_handle,
                                   m_gap.device_name_handle,
                                   0,
                                   m_gap.device_name_length,
                                   reinterpret_cast<const uint8_t*>(m_gap.device_name));
    }

    return true;
}

bool BLELocalDevice::setAppearance(enum BLEAppearance appearance) {
    if (!appearance) {
        return false;
    }

    m_gap.appearance = (uint16_t)appearance;

    if (m_state == BLE_LOCAL_STATE_READY) {
        aci_gatt_update_char_value(m_gap.service_handle,
                                   m_gap.appearance_handle,
                                   0,
                                   2,
                                   reinterpret_cast<const uint8_t*>(&m_gap.appearance));
    }
    
    return true;
}


bool BLELocalDevice::setSecurity(BLESecurity security) {
    m_gap.security = security;

    if (m_state == BLE_LOCAL_STATE_READY) {
        authentication();
    }

    return true;
}


bool BLELocalDevice::setPasskey(uint32_t passkey) {
    if ((passkey != BLE_PASSKEY_UNDEFINED) && (passkey > 999999)) {
        return false;
    }
  
    m_gap.passkey = passkey;
    
    return true;
}

bool BLELocalDevice::setBonding(bool bonding) {
    m_gap.bonding = bonding;

    if (m_state == BLE_LOCAL_STATE_READY) {
        authentication();
    }

    return true;
}

void BLELocalDevice::setConnectable(bool connectable) {
    m_peripheral.connectable = connectable;
}

void BLELocalDevice::setDiscoverable(BLEDiscoverable discoverable) {
    m_peripheral.discoverable = discoverable;
}

bool BLELocalDevice::setAdvertisingInterval(uint16_t minimumAdvertisingInterval, uint16_t maximumAdvertisingInterval) {

    if (minimumAdvertisingInterval > maximumAdvertisingInterval) {
        return false;
    }

    if ((minimumAdvertisingInterval < 0x0020) || (minimumAdvertisingInterval > 0x4000)) {
        return false;
    }

    if ((maximumAdvertisingInterval < 0x0020) || (maximumAdvertisingInterval > 0x4000)) {
        return false;
    }

    m_peripheral.advertising_interval_min = minimumAdvertisingInterval;
    m_peripheral.advertising_interval_max = maximumAdvertisingInterval;

    return true;
}

bool BLELocalDevice::setAdvertisingData(const BLEAdvertisingData &advertising_data) {
    if (!advertising_data.contains(BLE_AD_TYPE_FLAGS)) {
        if (advertising_data.length() > (BLE_AD_SIZE -3)) {
            return false;
        }
    }
    
    m_peripheral.advertising_data = advertising_data;
    m_peripheral.advertising_data.setFlags(BLE_AD_FLAGS_BR_EDR_NOT_SUPPORTED);

    return true;
}

void BLELocalDevice::setAdvertisingData() {
    m_peripheral.advertising_data.clear();
    m_peripheral.advertising_data.setFlags(BLE_AD_FLAGS_BR_EDR_NOT_SUPPORTED);
}

bool BLELocalDevice::setScanResponseData(const BLEAdvertisingData &scan_response_data) {
    m_peripheral.scan_response_data = scan_response_data;

    return true;
}

void BLELocalDevice::setScanResponseData() {
    m_peripheral.scan_response_data.clear();
}

bool BLELocalDevice::setLocalName(const char *localName) {
    return m_peripheral.advertising_data.setLocalName(localName);
}

bool BLELocalDevice::setServiceUuid(const BLEUuid &uuid) {
    return m_peripheral.advertising_data.setServiceUuid(uuid);
}

bool BLELocalDevice::setServiceSolicitation(const BLEUuid &uuid) {
    return m_peripheral.advertising_data.setServiceSolicitation(uuid);
}

bool BLELocalDevice::setServiceData(const BLEUuid &uuid, const uint8_t data[], int length) {
    return m_peripheral.advertising_data.setServiceData(uuid, data, length);
}

bool BLELocalDevice::setManufacturerData(const uint8_t data[], int length) {
    return m_peripheral.advertising_data.setManufacturerData(data, length);
}

bool BLELocalDevice::setBeacon(const BLEUuid &uuid, uint16_t major, uint16_t minor, int rssi) {
    return m_peripheral.advertising_data.setBeacon(uuid, major, minor, rssi);
}

bool BLELocalDevice::advertise() {
    uint32_t index;
    uint8_t bonded_device_count, resolving_device_count;
    bool connectable;
    BLEAdvertisingData advertising_data;
    Bonded_Device_Entry_t bonded_devices[((BLE_EVT_MAX_PARAM_LEN - 3) - 2) / sizeof(Bonded_Device_Entry_t)];
    tBleStatus status = BLE_STATUS_SUCCESS;

    if (m_state != BLE_LOCAL_STATE_READY) {
        return false;
    }

    connectable = m_machine.central.connected() ? false : m_peripheral.connectable;

    if (connectable && !m_server.active) {
        if (!setup()) {
            return false;
        }
    }

    status = authentication();

    if (status == BLE_STATUS_SUCCESS) {
        if ((m_options & BLE_OPTION_PRIVACY) || m_gap.bonding) {
            status = aci_gap_configure_whitelist();
        }
    }

    if (status == BLE_STATUS_SUCCESS) {
        if (m_options & BLE_OPTION_PRIVACY) {
            status = aci_gap_get_bonded_devices(&bonded_device_count, &bonded_devices[0]);

            if (status == BLE_STATUS_SUCCESS) {
                for (resolving_device_count = 0, index = 0; index < bonded_device_count; index++) {
                    if (bonded_devices[index].Address_Type == 0x00) {
                        bonded_devices[resolving_device_count++] = bonded_devices[index];
                    }
                }
                
                status = aci_gap_add_devices_to_resolving_list(resolving_device_count, (Whitelist_Identity_Entry_t*)&bonded_devices[0], true);
            }
        }
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        if (m_peripheral.discoverable == BLE_DISCOVERABLE_GENERAL) {
            status = aci_gap_set_discoverable((connectable ? ADV_IND : (m_peripheral.scan_response_data.length() ? ADV_SCAN_IND : ADV_NONCONN_IND)),
                                              m_peripheral.advertising_interval_min,
                                              m_peripheral.advertising_interval_max, 
                                              ((m_options & BLE_OPTION_PRIVACY) ? 0x02 : m_address.type()),
                                              NO_WHITE_LIST_USE,
                                              0, NULL,
                                              0, NULL,
                                              0, 0);
        } else {
            status = aci_gap_set_limited_discoverable((connectable ? ADV_IND : (m_peripheral.scan_response_data.length() ? ADV_SCAN_IND : ADV_NONCONN_IND)),
                                                      m_peripheral.advertising_interval_min,
                                                      m_peripheral.advertising_interval_max,
                                                      ((m_options & BLE_OPTION_PRIVACY)
                                                       ? ((m_options & BLE_OPTION_RANDOM_STATIC_ADDRESS) ? 0x03 : 0x02)
                                                       : ((m_options & BLE_OPTION_RANDOM_STATIC_ADDRESS) ? 0x01 : 0x00)),
                                                      NO_WHITE_LIST_USE,
                                                      0, NULL,
                                                      0, NULL,
                                                      0, 0);
        }
    }

    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gap_delete_ad_type(BLE_AD_TYPE_FLAGS);
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gap_delete_ad_type(BLE_AD_TYPE_TX_POWER_LEVEL);
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        m_machine.advertising_data = m_peripheral.advertising_data;
        m_machine.advertising_data.setFlags((BLEAdFlags)(FLAG_BIT_BR_EDR_NOT_SUPPORTED | ((m_peripheral.discoverable == BLE_DISCOVERABLE_GENERAL) ? FLAG_BIT_LE_GENERAL_DISCOVERABLE_MODE : FLAG_BIT_LE_LIMITED_DISCOVERABLE_MODE)));
        m_machine.advertising_data.setTxPowerLevel(m_connection.tx_power_level);
        m_machine.advertising_data.setConnectionInterval(m_connection.interval_min, m_connection.interval_max);

        status = hci_le_set_advertising_data(m_machine.advertising_data.length(), m_machine.advertising_data.data());
    }
    
    if (status == BLE_STATUS_SUCCESS) {
        status = hci_le_set_scan_response_data(m_peripheral.scan_response_data.length(), m_peripheral.scan_response_data.data());
    }
    
    if (status != BLE_STATUS_SUCCESS) {
        teardown();

        return false;
    }
    
    m_state = BLE_LOCAL_STATE_ADVERTISING;

    return true;
}

void BLELocalDevice::stopAdvertise() {
    if (m_state == BLE_LOCAL_STATE_ADVERTISING) {
        aci_gap_set_non_discoverable();

        m_state = BLE_LOCAL_STATE_READY;
    }
}

bool BLELocalDevice::advertising() {
    return (m_state == BLE_LOCAL_STATE_ADVERTISING);
}

BLEDevice BLELocalDevice::central() {
    if (m_machine.central.connected()) {
        return BLEDevice(&m_machine.central);
    }

    return BLEDevice();
}

bool BLELocalDevice::connected() {
    return m_machine.central.connected();
}

void BLELocalDevice::disconnect() {
    m_machine.central.disconnect();
}

bool BLELocalDevice::setScanInterval(uint16_t scanInterval) {
    if ((scanInterval < 0x0004) || (scanInterval > 0x4000)) {
        return false;
    }

    m_central.scan_interval = scanInterval;

    return true;
}

bool BLELocalDevice::setScanWindow(uint16_t scanWindow) {
    if ((scanWindow < 0x0004) || (scanWindow > 0x4000)) {
        return false;
    }

    m_central.scan_window = scanWindow;

    return true;
}

void BLELocalDevice::setScanType(BLEScanType scanType) {
    m_central.scan_type = scanType;
}

bool BLELocalDevice::scan(uint8_t scanMode) {
    uint8_t bonded_device_count, resolving_device_count;
    unsigned int index;
    Bonded_Device_Entry_t bonded_devices[((BLE_EVT_MAX_PARAM_LEN - 3) - 2) / sizeof(Bonded_Device_Entry_t)];
    tBleStatus status = BLE_STATUS_SUCCESS;

    if (m_state != BLE_LOCAL_STATE_READY) {
        return false;
    }

    m_central.scan_mode = scanMode;
    
    m_machine.report_index = 0;
    m_machine.report_read = 0;
    m_machine.report_write = 0;
    memset(&m_machine.report_history[0], 0, sizeof(m_machine.report_history));

    if (status == BLE_STATUS_SUCCESS) {
        if (m_options & BLE_OPTION_PRIVACY) {
            status = aci_gap_get_bonded_devices(&bonded_device_count, &bonded_devices[0]);

            if (status == BLE_STATUS_SUCCESS) {
                for (resolving_device_count = 0, index = 0; index < bonded_device_count; index++) {
                    if (bonded_devices[index].Address_Type == 0x00) {
                        bonded_devices[resolving_device_count++] = bonded_devices[index];
                    }
                }
                
                status = aci_gap_add_devices_to_resolving_list(resolving_device_count, (Whitelist_Identity_Entry_t*)&bonded_devices[0], true);
            }
        }
    }

    if (status == BLE_STATUS_SUCCESS) {
      status = aci_gap_start_observation_proc(m_central.scan_interval,
                                              m_central.scan_window,
                                              m_central.scan_type,
                                              ((m_options & BLE_OPTION_PRIVACY) ? 0x03 : ((m_options & BLE_OPTION_RANDOM_STATIC_ADDRESS) ? 0x01 : 0x00)),
                                              ((m_central.scan_mode & 1) ? 0x01 : 0x00),
                                              0);
    }

    if (status == BLE_STATUS_SUCCESS) {
        m_state = BLE_LOCAL_STATE_SCANNING;

        return true;
    }
    
    return false;
}

void BLELocalDevice::stopScan() {
    if (m_state == BLE_LOCAL_STATE_SCANNING) {
        aci_gap_terminate_gap_proc(GAP_OBSERVATION_PROC);

        m_state = BLE_LOCAL_STATE_READY;
    }
}

bool BLELocalDevice::scanning() {
    return (m_state == BLE_LOCAL_STATE_SCANNING);
}

unsigned int BLELocalDevice::reportCount() {
    uint32_t read, write;

    read = m_machine.report_read;
    write = m_machine.report_write;

    if (read != write) {
        read &= 0x7f;
        write &= 0x7f;

        if (write > read) {
            return (write - read);
        } else {
            return (write + (BLE_SCAN_QUEUE_ENTRIES - read));
        }
    }

    return 0;
}

bool BLELocalDevice::report(BLEAddress &address, int &rssi, BLEScanEvent &event, BLEAdvertisingData &data) {
    const BLEScanQueueEntry *report;
    uint32_t index;

    index = m_machine.report_read;
                        
    if (index != m_machine.report_write) {
        report = &m_machine.report_queue[index & 0x7f];

        address = BLEAddress(report->m_address, (BLEAddressType)(report->m_address_type));

        rssi = report->m_rssi;

        event = (BLEScanEvent)report->m_event_type;

        data = BLEAdvertisingData(report->m_data, report->m_length);

        index++;

        if ((index & 0x7f) == BLE_SCAN_QUEUE_ENTRIES) {
            index = (index ^ 0x80) & ~0x7f;
        }
        
        m_machine.report_read = index;

        return true;
    }

    return false;
}

BLEDevice BLELocalDevice::connect(BLEAddress address, uint32_t passkey, Callback callback) {
    BLEPeerDevice *device;
    int index;
    
    device = new BLEPeerDevice();
    
    if (!device) {
        return BLEDevice();
    }
    
    do {
        for (index = ((m_options & BLE_OPTION_ROLE_PERIPHERAL) ? 1 : 0); index < BLE_NUM_LINK; index++) {
            if (!m_machine.peer[index]) {
                break;
            }
        }

        if (index == BLE_NUM_LINK) {
            delete device;
            
            return BLEDevice();
        }
    } while (armv7m_atomic_cas((volatile uint32_t*)&m_machine.peer[index], (uint32_t)nullptr, (uint32_t)device) != (uint32_t)nullptr);

    device->m_index = index;
    device->m_address = address;

    device->m_done_callback = callback;

    device->m_machine.passkey = passkey;
    
    device->m_machine.params.interval_min = m_connection.interval_min;
    device->m_machine.params.interval_max = m_connection.interval_max;
    device->m_machine.params.latency = m_connection.latency;
    device->m_machine.params.timeout = m_connection.timeout;
    device->m_machine.params.min_ce_length = m_connection.min_ce_length;
    device->m_machine.params.max_ce_length = m_connection.max_ce_length;

    device->m_machine.connect_request = true;

    k_work_submit(&device->m_request_work);

    return BLEDevice(device);
}

bool BLELocalDevice::activate() {
    if (!m_server.active) {
        if (!setup()) {
            return false;
        }
    }

    return true;
}

unsigned int BLELocalDevice::serviceCount() {
    return m_server.count;
}

BLEService BLELocalDevice::service(unsigned int index) {
    class BLEServerService *serviceCurrent;

    for (serviceCurrent = m_server.children; serviceCurrent; serviceCurrent = serviceCurrent->m_sibling, index--) {
        if (index == 0) {
            return BLEService(serviceCurrent);
        }
    }
    
    return BLEService();
}

BLEService BLELocalDevice::service(const BLEUuid &uuid) {
    class BLEServerService *serviceCurrent;

    if (uuid) {
        for (serviceCurrent = m_server.children; serviceCurrent; serviceCurrent = serviceCurrent->m_sibling) {
            if (uuid == serviceCurrent->m_uuid) {
                return BLEService(serviceCurrent);
            }
        }
    }

    return BLEService();
}

bool BLELocalDevice::addService(BLEService &service) {
    class BLEServerService *serviceServer, *serviceCurrent;

    serviceServer = service.interface()->server();

    if (!serviceServer || serviceServer->m_parent || BLEHost.m_server.active) {
        return false;
    }

    serviceServer->reference();

    if (m_server.children) {
        for (serviceCurrent = m_server.children; serviceCurrent; serviceCurrent = serviceCurrent->m_sibling) {
            if (!serviceCurrent->m_sibling) {
                serviceCurrent->m_sibling = serviceServer;
                break;
            }
        }
    } else {
        m_server.children = serviceServer;
    }

    serviceServer->m_parent = this;
    
    m_server.count++;
    
    return true;
}

int BLELocalDevice::queryBondStorage(BLEAddress address[], int count) {
    uint8_t bonded_device_count;
    unsigned int index;
    Bonded_Device_Entry_t bonded_devices[((BLE_EVT_MAX_PARAM_LEN - 3) - 2) / sizeof(Bonded_Device_Entry_t)];
    tBleStatus status = BLE_STATUS_SUCCESS;

    if (m_state != BLE_LOCAL_STATE_READY) {
        return false;
    }

    status = aci_gap_get_bonded_devices(&bonded_device_count, &bonded_devices[0]);

    if (status == BLE_STATUS_SUCCESS) {
        if (bonded_device_count > count) {
            bonded_device_count = count;
        }

        for (index = 0; index < bonded_device_count; index++) {
            address[index] = BLEAddress(bonded_devices[index].Address, (BLEAddressType)bonded_devices[index].Address_Type);
        }

        return bonded_device_count;
    }

    return 0;
}

bool BLELocalDevice::removeBondStorage(const BLEAddress &address) {
    tBleStatus status = BLE_STATUS_SUCCESS;

    if (m_state != BLE_LOCAL_STATE_READY) {
        return false;
    }

    status = aci_gap_remove_bonded_device(address.type(), address.data());

    if (status == BLE_STATUS_SUCCESS) {
        return true;
    }
    
    return false;
}

bool BLELocalDevice::clearBondStorage() {
    tBleStatus status = BLE_STATUS_SUCCESS;

    if (m_state != BLE_LOCAL_STATE_READY) {
        return false;
    }
    
    status = aci_gap_clear_security_db();

    if (status == BLE_STATUS_SUCCESS) {
        return true;
    }
    
    return false;
}

bool BLELocalDevice::testTransmit(int channel, int length, BLEPayload payload, BLEPhy phy) {
    uint8_t status;

    if ((m_state != BLE_LOCAL_STATE_READY) || m_machine.central.connected()) {
        return false;
    }

    if ((channel < 0) || (channel > 39)) {
        return false;
    }

    if ((length < 0) || (length > 255)) {
        return false;
    }

    status = hci_le_transmitter_test_v2(channel, length, payload, phy);

    if (status != BLE_STATUS_SUCCESS) {
        return false;
    }

    m_state = BLE_LOCAL_STATE_TEST_TRANSMIT;
    
    return true;
}

bool BLELocalDevice::testReceive(int channel, BLEPhy phy, int modulation) {
    uint8_t status;

    if ((m_state != BLE_LOCAL_STATE_READY) || m_machine.central.connected()) {
        return false;
    }

    if ((channel < 0) || (channel > 39)) {
        return false;
    }

    if ((modulation < 0) || (modulation > 1)) {
        return false;
    }

    status = hci_le_receiver_test_v2(channel, phy, modulation);

    if (status != BLE_STATUS_SUCCESS) {
        return false;
    }

    m_state = BLE_LOCAL_STATE_TEST_RECEIVE;

    return true;
}    

uint32_t BLELocalDevice::testStop() {
    uint8_t status;
    uint32_t packets;
    
    if (!((m_state == BLE_LOCAL_STATE_TEST_TRANSMIT) || (m_state == BLE_LOCAL_STATE_TEST_RECEIVE))) {
        return false;
    }    

    packets = 0;
    
    status = hci_le_test_end((uint16_t*)&packets);

    if (m_state & BLE_LOCAL_STATE_TEST_TRANSMIT) {
        if (status == BLE_STATUS_SUCCESS) {
            status = aci_hal_le_tx_test_packet_number((uint32_t*)&packets);
        }
    }

    if (status != BLE_STATUS_SUCCESS) {
        packets = 0;
    }
    
    m_state = BLE_LOCAL_STATE_READY;

    return packets;
}

bool BLELocalDevice::testing() {
    return ((m_state == BLE_LOCAL_STATE_TEST_TRANSMIT) || (m_state == BLE_LOCAL_STATE_TEST_RECEIVE));
}

void BLELocalDevice::onStop(Callback callback) {
    m_stop_callback = callback;
}

void BLELocalDevice::onReport(Callback callback) {
    m_report_callback = callback;
}

void BLELocalDevice::onConnect(Callback callback) {
    m_connect_callback = callback;
}

void BLELocalDevice::onDisconnect(Callback callback) {
    m_disconnect_callback = callback;
}

/***************************************************************************************************/

tBleStatus BLELocalDevice::authentication() {
    uint8_t mitm, sc, min_key_size, max_key_size;
    bool use_passkey;
    tBleStatus status = BLE_STATUS_SUCCESS;

    switch (m_gap.security) {
    case BLE_SECURITY_UNENCRYPTED:
        mitm = MITM_PROTECTION_NOT_REQUIRED;
        sc = SC_PAIRING_OPTIONAL;
        min_key_size = 7;
        max_key_size = 16;
        use_passkey = false;
        break;
        
    case BLE_SECURITY_UNAUTHENTICATED:
        mitm = MITM_PROTECTION_NOT_REQUIRED;
        sc = SC_PAIRING_OPTIONAL;
        min_key_size = 7;
        max_key_size = 16;
        use_passkey = false;
        break;
        
    case BLE_SECURITY_AUTHENTICATED:
        mitm = MITM_PROTECTION_REQUIRED;
        sc = SC_PAIRING_OPTIONAL;
        min_key_size = 7;
        max_key_size = 16;
        use_passkey = true;
        break;

    case BLE_SECURITY_AUTHENTICATED_SECURE_CONNECTION:
        mitm = MITM_PROTECTION_REQUIRED;
        sc = SC_PAIRING_ONLY;
        min_key_size = 16;
        max_key_size = 16;
        use_passkey = true;
        break;

    default:
        return BLE_STATUS_FAILURE;
    }

    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gap_set_io_capability(use_passkey ? IO_CAP_DISPLAY_ONLY : IO_CAP_NO_INPUT_NO_OUTPUT);
    }

    if (status == BLE_STATUS_SUCCESS) {
        status = aci_gap_set_authentication_requirement(m_gap.bonding,
                                                        mitm,
                                                        sc,
                                                        KEYPRESS_NOT_SUPPORTED,
                                                        min_key_size,
                                                        max_key_size,
                                                        USE_FIXED_PIN_FOR_PAIRING_FORBIDDEN,
                                                        0,
                                                        (m_options & BLE_OPTION_RANDOM_STATIC_ADDRESS));
    }

    return status;
}

bool BLELocalDevice::setup() {
    BLEServerService *service;
    BLEServerCharacteristic *characteristic;
    unsigned int count;
    BLESecurity security_read, security_write, security_subscribe;
    uint8_t value_permissions, cccd_permissions, notify;
    tBleStatus status;

    if (m_server.active) {
        return true;
    }
    
    for (service = m_server.children; service; service = service->m_sibling) {

        for (count = 1, characteristic = service->m_children; characteristic; characteristic = characteristic->m_sibling) {
            count += 2;

            if (characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
                count += 1;
            }
        }

        status = aci_gatt_add_service(((service->m_uuid.type() == BLE_UUID_TYPE_16) ? UUID_TYPE_16 : UUID_TYPE_128),
                                      reinterpret_cast<const Service_UUID_t*>(service->m_uuid.data()),
                                      0x01,
                                      count,
                                      &service->m_handle);
        
        if (status != BLE_STATUS_SUCCESS) {
            teardown();

            return false;
        }
        
        service->m_end_group_handle = service->m_handle + count;

        service->reference();
        
#if (BLE_TRACE_SUPPORTED == 1)
        armv7m_rtt_printf("ADD-SERVICE \"%s\", %04x/%04x\r\n", uuid_string(service->m_uuid), service->m_handle, service->m_end_group_handle);
#endif

        for (characteristic = service->m_children; characteristic; characteristic = characteristic->m_sibling) {

            value_permissions = 0;
            cccd_permissions = 0;
            
            notify = GATT_DONT_NOTIFY_EVENTS;
            
            security_read = (BLESecurity)((characteristic->m_permissions >> 0) & 3);
            security_write = (BLESecurity)((characteristic->m_permissions >> 2) & 3);
            security_subscribe = (BLESecurity)((characteristic->m_permissions >> 4) & 3);

            if (characteristic->m_properties & (BLE_PROPERTY_READ)) {
                switch (security_read) {
                case BLE_SECURITY_UNENCRYPTED:
                    break;
                case BLE_SECURITY_UNAUTHENTICATED:
                    value_permissions |= ATTR_PERMISSION_ENCRY_READ;
                    break;
                case BLE_SECURITY_AUTHENTICATED:
                    value_permissions |= (ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_AUTHEN_READ);
                    break;
                case BLE_SECURITY_AUTHENTICATED_SECURE_CONNECTION:
                    value_permissions |= (ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_AUTHEN_READ);
                    break;
                };
            }

            if (characteristic->m_properties & (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE | BLE_PROPERTY_WRITE)) {
                notify |= GATT_NOTIFY_ATTRIBUTE_WRITE;

                switch (security_write) {
                case BLE_SECURITY_UNENCRYPTED:
                break;
                case BLE_SECURITY_UNAUTHENTICATED:
                    value_permissions |= ATTR_PERMISSION_ENCRY_WRITE;
                    break;
                case BLE_SECURITY_AUTHENTICATED:
                    value_permissions |= (ATTR_PERMISSION_ENCRY_WRITE | ATTR_PERMISSION_AUTHEN_WRITE);
                    break;
                case BLE_SECURITY_AUTHENTICATED_SECURE_CONNECTION:
                    value_permissions |= (ATTR_PERMISSION_ENCRY_WRITE | ATTR_PERMISSION_AUTHEN_WRITE);
                    break;
                };
            }
            
            if (characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
                notify |= GATT_NOTIFY_NOTIFICATION_COMPLETION;
                
                switch (security_subscribe) {
                case BLE_SECURITY_UNENCRYPTED:
                    break;
                case BLE_SECURITY_UNAUTHENTICATED:
                    cccd_permissions |= (ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_ENCRY_WRITE);
                    break;
                case BLE_SECURITY_AUTHENTICATED:
                    cccd_permissions |= (ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_AUTHEN_READ | ATTR_PERMISSION_ENCRY_WRITE | ATTR_PERMISSION_AUTHEN_WRITE);
                    break;
                case BLE_SECURITY_AUTHENTICATED_SECURE_CONNECTION:
                    cccd_permissions |= (ATTR_PERMISSION_ENCRY_READ | ATTR_PERMISSION_AUTHEN_READ | ATTR_PERMISSION_ENCRY_WRITE | ATTR_PERMISSION_AUTHEN_WRITE);
                    break;
                };
            }

            status = aci_gatt_add_char(service->m_handle,
                                       ((characteristic->m_uuid.type() == BLE_UUID_TYPE_16) ? UUID_TYPE_16 : UUID_TYPE_128),
                                       reinterpret_cast<const Char_UUID_t*>(characteristic->m_uuid.data()),
                                       characteristic->m_value_size,
                                       characteristic->m_properties,
                                       value_permissions,
                                       notify,
                                       7,
                                       !characteristic->m_fixed_length,
                                       &characteristic->m_handle);

            if (status != BLE_STATUS_SUCCESS) {
                teardown();

                return false;
            }

            characteristic->reference();

            if (characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
                status = aci_gatt_set_security_permission(service->m_handle,
                                                          (characteristic->m_handle + BLE_CCCD_ATTRIB_HANDLE_OFFSET),
                                                          cccd_permissions);

                if (status != BLE_STATUS_SUCCESS) {
                    teardown();

                    return false;
                }
            }
            
#if (BLE_TRACE_SUPPORTED == 1)
            armv7m_rtt_printf("ADD-CHARACTERISTIC \"%s\", %04x\r\n", uuid_string(characteristic->m_uuid), characteristic->m_handle);
#endif
        }
    }

    m_server.active = true;

    return true;
}

void BLELocalDevice::teardown() {
    BLEServerService *service, *service_next;
    BLEServerCharacteristic *characteristic, *characteristic_next;
    BLEServerCharacteristic *notify, *request_submit, *request_next, *request_head, *request_tail;
    Callback callback;

    m_server.active = false;
    
    if (m_machine.request_submit) {
        request_submit = (class BLEServerCharacteristic *)armv7m_atomic_swap((volatile uint32_t*)&m_machine.request_submit, (uint32_t)nullptr);
        
        for (request_head = nullptr, request_tail = request_submit; request_submit != nullptr; request_submit = request_next)
        {
            request_next = request_submit->m_request;
            
            request_submit->m_request = request_head;
            
            request_head = request_submit;
        }
        
        if (!m_machine.request_head)
        {
            m_machine.request_head = request_head;
        }
        else
        {
            m_machine.request_tail->m_request = request_head;
        }
        
        m_machine.request_tail = request_tail;
    }
    
    while (m_machine.request_head) {
        notify = m_machine.request_head;
        
        if (m_machine.request_head == m_machine.request_tail)
        {
            m_machine.request_head = nullptr;
            m_machine.request_tail = nullptr;
        }
        else
        {
            m_machine.request_head = notify->m_request;
        }
        
        callback = notify->m_done_callback;
        
        notify->m_status = BLE_STATUS_FAILURE; // HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE;
        
        callback();
    }

    armv7m_atomic_and(&m_machine.request, ~(BLE_REQUEST_GATT_NOTIFY_VALUE | BLE_REQUEST_GATT_SET_VALUE));
    
    armv7m_atomic_andh(&m_machine.event, ~(BLE_EVENT_GATT_SERVER_CONFIRMATION | BLE_EVENT_GATT_NOTIFICATION_COMPLETE));
    
    if (m_machine.notify_current) {
        notify = m_machine.notify_current;
        
        m_machine.notify_current = nullptr;

        callback = notify->m_done_callback;
        
        notify->m_status = BLE_STATUS_FAILURE; // HCI_REMOTE_USER_TERMINATED_CONNECTION_ERR_CODE;
        
        callback();
    }

    m_machine.set_current = nullptr;

    for (service = m_server.children; service; service = service_next) {
        service_next = service->m_sibling;

        for (characteristic = service->m_children; characteristic; characteristic = characteristic_next) {
            characteristic_next = characteristic->m_sibling;

            if (characteristic->m_config) {
                characteristic->m_config = 0;

                characteristic->m_subscribe_callback();
            }
            
            characteristic->m_control = 0;
            
            if (characteristic->m_handle) {
                aci_gatt_del_char(service->m_handle, characteristic->m_handle);
            
                characteristic->m_handle = 0x0000;
            }
            
            characteristic->unreference();
        }

        if (service->m_handle) {
            aci_gatt_del_service(service->m_handle);

            service->m_handle = 0x0000;
            service->m_end_group_handle = 0x0000;
        }
        
        service->unreference();
    }
}

/***************************************************************************************************/

bool BLELocalDevice::complete(uint16_t handle, uint8_t role, const uint8_t *address, uint8_t type, uint16_t interval, uint16_t latency, uint16_t timeout) {
    BLEPeerDevice *device;
    int index;
    
    if (role) {
        device = &m_machine.central;

        if (m_machine.peer[0]) {
            return false;
        }
        
        m_machine.peer[0] = device;

        device->m_handle = handle;
        device->m_address = BLEAddress(address, (BLEAddressType)(type));
    } else {
        device = nullptr;
        
        for (index = ((m_options & BLE_OPTION_ROLE_PERIPHERAL) ? 1 : 0); index < BLE_NUM_LINK; index++) {
            if (m_machine.peer[index] && (m_machine.peer[index]->m_address == BLEAddress(address, (BLEAddressType)(type & 1)))) {
                device = m_machine.peer[index];
                break;
            }
        }

        if (!device) {
            return false;
        }

        device->m_handle = handle;
    }
    
    device->m_mtu = BLE_MIN_ATT_MTU;
    device->m_connection.tx_phy = BLE_PHY_1M;
    device->m_connection.rx_phy = BLE_PHY_1M;
    device->m_connection.interval = interval;
    device->m_connection.latency = latency;
    device->m_connection.timeout = timeout;
    
    if (role) {
        device->m_machine.passkey = m_gap.passkey;
      
        if (m_gap.security != BLE_SECURITY_UNENCRYPTED) {
            armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GAP_PERIPHERAL_SECURITY);
            armv7m_atomic_orh(&device->m_machine.event, BLE_EVENT_GAP_PAIRING_COMPLETE);
        }

        if ((m_connection.tx_phy != BLE_PHY_1M) || (m_connection.rx_phy != BLE_PHY_1M)) {
            armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_HCI_LE_SET_PHY);
            armv7m_atomic_orh(&device->m_machine.event, BLE_EVENT_HCI_LE_READ_PHY);
        }

        if ((interval < m_connection.interval_min) || (interval > m_connection.interval_max)) {
            armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_L2CAP_CONNECTION_PARAMETER_UPDATE);
            armv7m_atomic_orh(&device->m_machine.event, BLE_EVENT_HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT);
        }
        
        m_state = BLE_LOCAL_STATE_READY;
    } else {
        if (m_gap.security != BLE_SECURITY_UNENCRYPTED) {
            armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GAP_CENTRAL_SECURITY);
            armv7m_atomic_orh(&device->m_machine.event, BLE_EVENT_GAP_PAIRING_COMPLETE);
        }

        if (device->m_mtu != m_mtu) {
            armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GATT_MTU_EXCHANGE);
            armv7m_atomic_orh(&device->m_machine.event, BLE_EVENT_GATT_MTU_EXCHANGE);
        }
    }

    if (!device->m_machine.event) {
        connect(device);
    }
    
    return true;
}

void BLELocalDevice::connect(BLEPeerDevice *device) {
    device->m_machine.connected = true;

    if ((m_options & BLE_OPTION_ROLE_PERIPHERAL) && (device->m_index == 0)) {
        m_connect_callback();
    } else {
        device->m_done_callback();
    }
};

void BLELocalDevice::disconnect(BLEPeerDevice *device) {
    BLEClientService *service, *service_next;
    BLEClientCharacteristic *characteristic, *characteristic_next;
    
    m_machine.peer[device->m_index] = nullptr;

    device->m_handle = BLE_PEER_DEVICE_NOT_CONNECTED;

    device->m_machine.request = 0;
    device->m_machine.event = 0;
    
    if (device->m_machine.connected) {
        device->m_machine.connected = false;

        device->m_client.discovered = false;
        device->m_client.count = 0;
        
        for (service = device->m_client.children; service; service = service_next) {
            service_next = service->m_sibling;
            
            for (characteristic = service->m_children; characteristic; characteristic = characteristic_next) {
                characteristic_next = characteristic->m_sibling;

                characteristic->m_handle = 0;
                characteristic->m_value_handle = 0;
                characteristic->m_config_handle = 0;
                characteristic->m_parent = nullptr;
                characteristic->m_sibling = nullptr;

                characteristic->unreference();
            }
                
            service->m_count = 0;
            service->m_handle = 0;
            service->m_end_group_handle = 0;
            service->m_parent = nullptr;
            service->m_sibling = nullptr;
            service->m_children = nullptr;
            
            service->unreference();
        }
        
        device->m_disconnect_callback();

        if ((m_options & BLE_OPTION_ROLE_PERIPHERAL) && (device->m_index == 0)) {
            m_disconnect_callback();
        }
    }
    
    device->unreference();
}

void BLELocalDevice::done() {
    Callback callback;
    uint8_t busy, control;

#if (BLE_TRACE_SUPPORTED_EXT == 1)
    armv7m_rtt_printf("DONE-ENTER %02x/%08x/%04x\r\n", m_machine.busy, m_machine.request, m_machine.event);
#endif

    busy = m_machine.busy;

    m_machine.busy = BLE_BUSY_NONE;

    switch (busy) {
    case BLE_BUSY_NONE: {
        break;
    }

    case BLE_BUSY_HCI_LE_SET_PHY: {
        BLEPeerDevice *device = m_machine.device;

        armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_HCI_LE_READ_PHY);
        break;
    }

    case BLE_BUSY_HCI_LE_READ_PHY: {
        const hci_le_read_phy_rp0 *rparam = (const hci_le_read_phy_rp0*)&m_machine.command.rparam;
        BLEPeerDevice *device = m_machine.device;
        
        if (rparam->Status == BLE_STATUS_SUCCESS) {
            device->m_connection.tx_phy = rparam->TX_PHY;
            device->m_connection.rx_phy = rparam->RX_PHY;
        }
                
        armv7m_atomic_andh(&device->m_machine.event, ~BLE_EVENT_HCI_LE_READ_PHY);
        
        if (!device->m_machine.connected && !device->m_machine.event) {
            connect(device);
        }
        break;
    }
        
    case BLE_BUSY_L2CAP_CONNECTION_PARAMETER_UPDATE: {
        break;
    }

    case BLE_BUSY_GAP_CREATE_CONNECTION: {
        const aci_gap_create_connection_rp0 *rparam = (const aci_gap_create_connection_rp0*)m_machine.command.rparam;
        BLEPeerDevice *device = m_machine.device;

        if (rparam->Status != BLE_STATUS_SUCCESS) {
            device->m_done_callback();
        }
        break;
    }
      
    case BLE_BUSY_GAP_TERMINATE: {
        break;
    }

    case BLE_REQUEST_GAP_PASSKEY: {
        break;
    }
        
    case BLE_BUSY_GAP_PERIPHERAL_SECURITY: {
        const aci_gap_peripheral_security_req_rp0 *rparam = (const aci_gap_peripheral_security_req_rp0*)m_machine.command.rparam;
        BLEPeerDevice *device = m_machine.device;

        if (rparam->Status != BLE_STATUS_SUCCESS) {
            armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GAP_TERMINATE);
        }
        break;
    }

    case BLE_BUSY_GAP_CENTRAL_SECURITY: {
        const aci_gap_send_pairing_req_rp0 *rparam = (const aci_gap_send_pairing_req_rp0*)m_machine.command.rparam;
        BLEPeerDevice *device = m_machine.device;

        if (rparam->Status != BLE_STATUS_SUCCESS) {
            armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GAP_TERMINATE);
        }
        break;
    }
      
    case BLE_BUSY_GAP_ALLOW_REBOND: {
        const aci_gap_peripheral_security_req_rp0 *rparam = (const aci_gap_peripheral_security_req_rp0*)m_machine.command.rparam;
        BLEPeerDevice *device = m_machine.device;

        if (rparam->Status != BLE_STATUS_SUCCESS) {
            armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GAP_TERMINATE);
        }
        break;
    }

    case BLE_BUSY_GATT_NOTIFY_VALUE: {
        const aci_gatt_update_char_value_ext_rp0 *rparam = (const aci_gatt_update_char_value_ext_rp0*)m_machine.command.rparam;

        if (m_machine.notify_current) {
            BLEServerCharacteristic *notify = m_machine.notify_current;
            
            if (rparam->Status != BLE_STATUS_INSUFFICIENT_RESOURCES) {
                if (rparam->Status == BLE_STATUS_SUCCESS) {
                    /* NOTIFY success
                     */
                    m_machine.notify_offset += m_machine.notify_count;
                        
                    if (m_machine.notify_offset == m_machine.notify_length) {
                        if (m_machine.notify_type) {
                            if (m_machine.notify_type & 0x02) {
                                armv7m_atomic_orh(&m_machine.event, BLE_EVENT_GATT_SERVER_CONFIRMATION);
                            } else {
                                armv7m_atomic_orh(&m_machine.event, BLE_EVENT_GATT_NOTIFICATION_COMPLETE);
                            }
                        } else {
                            m_machine.notify_current = nullptr;

                            callback = notify->m_done_callback;
                                    
                            notify->m_status = BLE_STATUS_SUCCESS;

                            control = armv7m_atomic_andb(&notify->m_control, ~BLE_SERVER_CONTROL_NOTIFY);
                        
                            if ((control & (BLE_SERVER_CONTROL_MODIFIED | BLE_SERVER_CONTROL_WRITTEN)) == BLE_SERVER_CONTROL_MODIFIED) {
                                do {
                                    control = notify->m_control;
                                    
                                    if (control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET)) {
                                        break;
                                    }
                                } while (armv7m_atomic_casb(&notify->m_control, control, (control | BLE_SERVER_CONTROL_WRITTEN)) != control);
                                
                                if (!(control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET))) {
#if (BLE_TRACE_SUPPORTED == 1)
                                    armv7m_rtt_printf("ATTRIB-WRITTEN (\"%s\")\r\n", uuid_string(notify->m_uuid));
#endif
                                    notify->m_value_length = notify->m_write_length;
                                    notify->m_write_length = 0;
                                    
                                    memcpy(&notify->m_value_data[0], &notify->m_value_data[notify->m_value_size], notify->m_value_length);
                                    
                                    notify->m_write_callback();
                                }
                            }

#if (BLE_TRACE_SUPPORTED == 1)
                            armv7m_rtt_printf("ATTRIB-NOTIFY-DONE %02x (\"%s\")\r\n", rparam->Status, uuid_string(notify->m_uuid));
#endif

                            callback();
                        }
                    } else {
                        /* NOTIFY continue
                         */
                        armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_NOTIFY_VALUE);
                    }
                } else {
                    /* NOTIFY failure
                     */
                    m_machine.notify_current = nullptr;

                    callback = notify->m_done_callback;
                    
                    notify->m_status = BLE_STATUS_FAILURE;

                    control = armv7m_atomic_andb(&notify->m_control, ~BLE_SERVER_CONTROL_NOTIFY);
                        
                    if ((control & (BLE_SERVER_CONTROL_MODIFIED | BLE_SERVER_CONTROL_WRITTEN)) == BLE_SERVER_CONTROL_MODIFIED) {
                        do {
                            control = notify->m_control;
                            
                            if (control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET)) {
                                break;
                            }
                        } while (armv7m_atomic_casb(&notify->m_control, control, (control | BLE_SERVER_CONTROL_WRITTEN)) != control);
                        
                        if (!(control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET))) {
#if (BLE_TRACE_SUPPORTED == 1)
                            armv7m_rtt_printf("ATTRIB-WRITTEN (\"%s\")\r\n", uuid_string(notify->m_uuid));
#endif
                            notify->m_value_length = notify->m_write_length;
                            notify->m_write_length = 0;
        
                            memcpy(&notify->m_value_data[0], &notify->m_value_data[notify->m_value_size], notify->m_value_length);

                            notify->m_write_callback();
                        }
                    }

#if (BLE_TRACE_SUPPORTED == 1)
                    armv7m_rtt_printf("ATTRIB-NOTIFY-DONE %02x (\"%s\")\r\n", rparam->Status, uuid_string(notify->m_uuid));
#endif

                    callback();
                }
            } else {
                /* NOTIFY insufficient resources.
                 */
                armv7m_atomic_orh(&m_machine.event, BLE_EVENT_GATT_TX_POOL_AVAILABLE);
            }
        }
        break;
    }

    case BLE_BUSY_GATT_SET_VALUE: {
        const aci_gatt_update_char_value_ext_rp0 *rparam = (const aci_gatt_update_char_value_ext_rp0*)m_machine.command.rparam;

        if (m_machine.set_current) {
            BLEServerCharacteristic *set = m_machine.set_current;

            if (rparam->Status == BLE_STATUS_SUCCESS) {
                m_machine.set_offset += m_machine.set_count;
                    
                if (m_machine.set_offset == m_machine.set_length) {
                    /* SET success.
                     */
                    m_machine.set_current = nullptr;

                    do {
                        control = set->m_control;

                        if (control & BLE_SERVER_CONTROL_CHANGED) {
                            break;
                        }
                    } while (armv7m_atomic_casb(&set->m_control, control, (control & ~BLE_SERVER_CONTROL_SET)));
#if (BLE_TRACE_SUPPORTED == 1)
                    armv7m_rtt_printf("ATTRIB-SET-DONE %02x (\"%s\")\r\n", BLE_STATUS_SUCCESS, uuid_string(set->m_uuid));
#endif

                    if ((control & (BLE_SERVER_CONTROL_MODIFIED | BLE_SERVER_CONTROL_WRITTEN)) == BLE_SERVER_CONTROL_MODIFIED) {
                        do {
                            control = set->m_control;
                            
                            if (control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET)) {
                                break;
                            }
                        } while (armv7m_atomic_casb(&set->m_control, control, (control | BLE_SERVER_CONTROL_WRITTEN)) != control);
                        
                        if (!(control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET))) {
#if (BLE_TRACE_SUPPORTED == 1)
                            armv7m_rtt_printf("ATTRIB-WRITTEN (\"%s\")\r\n", uuid_string(set->m_uuid));
#endif
                            set->m_value_length = set->m_write_length;
                            set->m_write_length = 0;
        
                            memcpy(&set->m_value_data[0], &set->m_value_data[set->m_value_size], set->m_value_length);

                            set->m_write_callback();
                        }
                    }
                } else {
                    /* SET continue.
                     */
                    armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_SET_VALUE);
                }
            } else {
                    /* SET failure.
                     */
                m_machine.set_current = nullptr;

                do {
                    control = set->m_control;
                    
                    if (control & BLE_SERVER_CONTROL_CHANGED) {
                        break;
                    }
                } while (armv7m_atomic_casb(&set->m_control, control, (control & ~BLE_SERVER_CONTROL_SET)));
                
#if (BLE_TRACE_SUPPORTED == 1)
                armv7m_rtt_printf("ATTRIB-SET-DONE %02x (\"%s\")\r\n", rparam->Status, uuid_string(set->m_uuid));
#endif

                if ((control & (BLE_SERVER_CONTROL_MODIFIED | BLE_SERVER_CONTROL_WRITTEN)) == BLE_SERVER_CONTROL_MODIFIED) {
                    do {
                        control = set->m_control;
                        
                        if (control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET)) {
                            break;
                        }
                    } while (armv7m_atomic_casb(&set->m_control, control, (control | BLE_SERVER_CONTROL_WRITTEN)) != control);
                    
                    if (!(control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET))) {
#if (BLE_TRACE_SUPPORTED == 1)
                        armv7m_rtt_printf("ATTRIB-WRITTEN (\"%s\")\r\n", uuid_string(set->m_uuid));
#endif

                        set->m_value_length = set->m_write_length;
                        set->m_write_length = 0;
        
                        memcpy(&set->m_value_data[0], &set->m_value_data[set->m_value_size], set->m_value_length);

                        set->m_write_callback();
                    }
                }
            }
        }
        break;
    }

    case BLE_BUSY_GATT_ALLOW_READ: {
        break;
    }
           
    case BLE_BUSY_GATT_MTU_EXCHANGE: {
        break;
    }

    case BLE_BUSY_GATT_DISCOVER_SERVICE_BY_UUID: {
        aci_gatt_disc_primary_service_by_uuid_rp0 *rparam = (aci_gatt_disc_primary_service_by_uuid_rp0*)m_machine.command.rparam;
        BLEPeerDevice *device = m_machine.device;

        if (rparam->Status == BLE_STATUS_SUCCESS) {
            armv7m_atomic_orh(&device->m_machine.event, BLE_EVENT_GATT_PROC_COMPLETE);
        } else {
            device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;
            device->m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;

            device->m_done_callback();
        }
        break;
    }
        
    case BLE_BUSY_GATT_DISCOVER_SERVICES: {
        aci_gatt_disc_all_primary_services_rp0 *rparam = (aci_gatt_disc_all_primary_services_rp0*)m_machine.command.rparam;
        BLEPeerDevice *device = m_machine.device;

        if (rparam->Status == BLE_STATUS_SUCCESS) {
            armv7m_atomic_orh(&device->m_machine.event, BLE_EVENT_GATT_PROC_COMPLETE);
        } else {
            device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;
            device->m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;

            device->m_done_callback();
        }
        break;
    }

    case BLE_BUSY_GATT_DISCOVER_CHARACTERISTICS: {
        aci_gatt_disc_all_char_of_service_rp0 *rparam = (aci_gatt_disc_all_char_of_service_rp0*)m_machine.command.rparam;
        BLEPeerDevice *device = m_machine.device;

        if (rparam->Status == BLE_STATUS_SUCCESS) {
            armv7m_atomic_orh(&device->m_machine.event, BLE_EVENT_GATT_PROC_COMPLETE);
        } else {
            device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;
            device->m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;

            device->m_done_callback();
        }
        break;
    }

    case BLE_BUSY_GATT_DISCOVER_DESCRIPTORS: {
        aci_gatt_disc_all_char_desc_rp0 *rparam = (aci_gatt_disc_all_char_desc_rp0*)m_machine.command.rparam;
        BLEPeerDevice *device = m_machine.device;

        if (rparam->Status == BLE_STATUS_SUCCESS) {
            armv7m_atomic_orh(&device->m_machine.event, BLE_EVENT_GATT_PROC_COMPLETE);
        } else {
            device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;
            device->m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;
                                    
            device->m_done_callback();
        }
        break;
    }
            
    case BLE_BUSY_GATT_CONFIRM_INDICATION: { 
        break;
    }

    case BLE_BUSY_GATT_READ_VALUE: {
        break;
    }

    case BLE_BUSY_GATT_READ_VALUE_LONG: {
        break;
    }

    case BLE_BUSY_GATT_WRITE_VALUE: {
        break;
    }

    case BLE_BUSY_GATT_WRITE_VALUE_LONG: {
        break;
    }

    case BLE_BUSY_GATT_WRITE_VALUE_WITHOUT_RESPONSE: {
        break;
    }

    case BLE_BUSY_GATT_WRITE_CONFIG: {
        break;
    }
        
    default:
        break;
    }

#if (BLE_TRACE_SUPPORTED_EXT == 1)
    armv7m_rtt_printf("DONE-LEAVE %02x/%08x/%04x\r\n", m_machine.busy, m_machine.request, m_machine.event);
#endif

    request();
}

void BLELocalDevice::event() {
    const hci_uart_pckt *event;    
    const hci_event_pckt *hci_event;
    const evt_le_meta_event *hci_le_event;
    const evt_blue_aci *hci_vs_event;
    Callback callback;
    uint8_t control;
    
#if (BLE_TRACE_SUPPORTED_EXT == 1)
    armv7m_rtt_printf("EVENT-ENTER %02x/%08x/%04x\r\n", m_machine.busy, m_machine.request, m_machine.event);
#endif

    event = (const hci_uart_pckt*)stm32wb_ipcc_ble_event();

    if (event) {
        do {
            if (event->type == HCI_EVENT_PKT_TYPE) {
                hci_event = (const hci_event_pckt*)&event->data[0];

#if 0                
#if (BLE_TRACE_SUPPORTED == 1)
                armv7m_rtt_printf("HCI_EVENT %08x\r\n", hci_event->evt);
#endif
#endif
                
                switch (hci_event->evt) {

                case HCI_DISCONNECTION_COMPLETE_EVENT: {
                    const hci_disconnection_complete_event_rp0 *eparam = (const hci_disconnection_complete_event_rp0*)&hci_event->data[0];
                    BLEPeerDevice *device;
                    int index;

                    for (index = 0; index < BLE_NUM_LINK; index++) {
                        if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                            device = m_machine.peer[index];

                            disconnect(device);

                            if (m_machine.terminate_waiting) {
                                k_event_send(m_machine.terminate_waiting, WIRING_EVENT_TRANSIENT);
                            }
                            break;
                        }
                    }
                    break;
                }
                    
                case HCI_ENCRYPTION_CHANGE_EVENT:
                case HCI_READ_REMOTE_VERSION_INFORMATION_COMPLETE_EVENT:
                case HCI_HARDWARE_ERROR_EVENT:
                case HCI_ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT:
                    break;
                    
                case HCI_LE_META_EVENT:
                    hci_le_event = (const evt_le_meta_event*)&hci_event->data[0];

#if 0                    
#if (BLE_TRACE_SUPPORTED == 1)
                    armv7m_rtt_printf("HCI_LE_EVENT %08x\r\n", hci_le_event->subevent);
#endif
#endif
                    
                    switch (hci_le_event->subevent) {
                        
                    case HCI_LE_CONNECTION_COMPLETE_EVENT: {
                        const hci_le_connection_complete_event_rp0 *eparam = (const hci_le_connection_complete_event_rp0*)&hci_le_event->data[0];

                        if (eparam->Status == BLE_STATUS_SUCCESS) {
                            if (!complete(eparam->Connection_Handle, eparam->Role, eparam->Peer_Address, eparam->Peer_Address_Type, eparam->Conn_Interval, eparam->Conn_Latency, eparam->Supervision_Timeout)) {
                                if (eparam->Role) {
                                    if (m_state == BLE_LOCAL_STATE_ADVERTISING) {
                                        m_state = BLE_LOCAL_STATE_READY;

#if (BLE_TRACE_SUPPORTED == 1)
                                        armv7m_rtt_printf("REQUEST-STOP\r\n");
#endif
                                
                                        m_stop_callback();
                                    }
                                }
                            }
                        } else {
                            if (eparam->Role) {
                                if (m_state == BLE_LOCAL_STATE_ADVERTISING) {
                                    m_state = BLE_LOCAL_STATE_READY;
                                    
#if (BLE_TRACE_SUPPORTED == 1)
                                    armv7m_rtt_printf("REQUEST-STOP\r\n");
#endif
                                    
                                    m_stop_callback();
                                }
                            }
                        }
                        break;
                    }

                    case HCI_LE_ADVERTISING_REPORT_EVENT: {
                        const hci_le_advertising_report_event_rp0 *eparam = (const hci_le_advertising_report_event_rp0*)&hci_le_event->data[0];
                        BLEScanHistoryEntry history;
                        BLEScanQueueEntry *report;
                        uint32_t index, slot, write, count, length;
                        const uint8_t *data;
                        uint8_t event;
                        bool accept, connectable;
                        
                        count = eparam->Num_Reports;
                        data = &eparam->Num_Reports + 1;

                        write = m_machine.report_write;
                        
                        if (count) {
                            do {
                                accept = true;
                                connectable = ((data[0] == HCI_ADV_EVT_TYPE_ADV_IND) || (data[0] == HCI_ADV_EVT_TYPE_ADV_DIRECT_IND));
                                event = ((data[0] == HCI_ADV_EVT_TYPE_SCAN_RSP) ? BLE_SCAN_EVENT_RESPONSE : ((data[0] != HCI_ADV_EVT_TYPE_ADV_DIRECT_IND) ? BLE_SCAN_EVENT_ADVERTISEMENT : 0));
                                
                                length = data[8];

                                history.m_accept = false;
                                history.m_event_type = 0;
                                history.m_address_type = data[1];
                                history.m_address[0] = data[2];
                                history.m_address[1] = data[3];
                                history.m_address[2] = data[4];
                                history.m_address[3] = data[5];
                                history.m_address[4] = data[6];
                                history.m_address[5] = data[7];

                                for (index = 0; index < BLE_SCAN_HISTORY_ENTRIES; index++) {
                                    slot = (m_machine.report_index + index) & (BLE_SCAN_HISTORY_ENTRIES-1);
                                  
                                    if ((m_machine.report_history[slot].m_address_type == history.m_address_type) && !memcmp(&m_machine.report_history[slot].m_address[0], &history.m_address[0], BLE_ADDRESS_SIZE)) {
                                        if (connectable) {
                                            m_machine.report_history[slot].m_event_type |= BLE_SCAN_EVENT_CONNECTABLE;
                                        } else {
                                            if ((m_central.scan_mode & BLE_SCAN_MODE_CONNECTABLE) && !(m_machine.report_history[slot].m_event_type & BLE_SCAN_EVENT_CONNECTABLE)) {
                                                accept = false;
                                            }
                                        }
                                      
                                        break;
                                    }
                                }
                                
                                if (index == BLE_SCAN_HISTORY_ENTRIES) {
                                    /* Avoid adding a new history entry for something that will not be accepted.
                                     */
                                    if ((m_central.scan_mode & BLE_SCAN_MODE_CONNECTABLE) && !connectable) {
                                        accept = false;
                                    } else {
                                        slot = m_machine.report_index;

                                        memcpy(&m_machine.report_history[slot], &history, sizeof(BLEScanHistoryEntry));

                                        if (connectable) {
                                            m_machine.report_history[slot].m_event_type |= BLE_SCAN_EVENT_CONNECTABLE;
                                        }
                                        
                                        m_machine.report_index = (m_machine.report_index +1) & (BLE_SCAN_HISTORY_ENTRIES-1);
                                    }
                                }
                                
                                if (accept && (m_central.scan_mode & BLE_SCAN_MODE_WITHOUT_DUPLICATES)) {
                                    if (m_machine.report_history[slot].m_event_type & event) {
                                        accept = false;
                                    }
                                }

                                /* Do not pass on empty scan responses.
                                 */
                                if (accept && (event & BLE_SCAN_EVENT_RESPONSE) && !length) {
                                  accept = false;
                                }
                                
                                if (accept) {
                                    m_machine.report_history[slot].m_accept = true;
                                    m_machine.report_history[slot].m_event_type |= event;
                                  
                                    if ((write ^ 0x80) != m_machine.report_read) {
                                        report = &m_machine.report_queue[write & 0x7f];
                                        
                                        report->m_accept = true;
                                        report->m_event_type = event | (m_machine.report_history[slot].m_event_type & BLE_SCAN_EVENT_CONNECTABLE);;
                                        report->m_address_type = data[1];
                                        report->m_address[0] = data[2];
                                        report->m_address[1] = data[3];
                                        report->m_address[2] = data[4];
                                        report->m_address[3] = data[5];
                                        report->m_address[4] = data[6];
                                        report->m_address[5] = data[7];
                                        report->m_length = length;
                                        memcpy(&report->m_data[0], &data[9], length);
                                        report->m_rssi = (int8_t)data[9 + length];
                                        
                                        write++;
                                        
                                        if ((write & 0x7f) == BLE_SCAN_QUEUE_ENTRIES) {
                                            write = (write ^ 0x80) & ~0x7f;
                                        }
                                        
                                        m_machine.report_write = write;
                                    }
                                }
                                
                                data += (10 + length);
                                count--;
                            } while (count);

                            m_report_callback();
                        }
                        break;
                    }
                        
                    case HCI_LE_ENHANCED_CONNECTION_COMPLETE_EVENT: {
                        const hci_le_enhanced_connection_complete_event_rp0 *eparam = (const hci_le_enhanced_connection_complete_event_rp0*)&hci_le_event->data[0];

                        if (eparam->Status == BLE_STATUS_SUCCESS) {
                            if (!complete(eparam->Connection_Handle, eparam->Role, eparam->Peer_Address, eparam->Peer_Address_Type, eparam->Conn_Interval, eparam->Conn_Latency, eparam->Supervision_Timeout)) {
                                if (eparam->Role) {
                                    if (m_state == BLE_LOCAL_STATE_ADVERTISING) {
                                        m_state = BLE_LOCAL_STATE_READY;

#if (BLE_TRACE_SUPPORTED == 1)
                                        armv7m_rtt_printf("REQUEST-STOP\r\n");
#endif
                                
                                        m_stop_callback();
                                    }
                                }
                            }
                        } else {
                            if (eparam->Role) {
                                if (m_state == BLE_LOCAL_STATE_ADVERTISING) {
                                    m_state = BLE_LOCAL_STATE_READY;
                                    
#if (BLE_TRACE_SUPPORTED == 1)
                                    armv7m_rtt_printf("REQUEST-STOP\r\n");
#endif
                                    
                                    m_stop_callback();
                                }
                            }
                        }
                        break;
                    }
                        
                    case HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT: {
                        const hci_le_connection_update_complete_event_rp0 *eparam = (const hci_le_connection_update_complete_event_rp0*)&hci_le_event->data[0];
                        BLEPeerDevice *device;
                        int index;

                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];
                        
                                device->m_connection.interval = eparam->Conn_Interval;
                                device->m_connection.latency = eparam->Conn_Latency;
                                device->m_connection.timeout = eparam->Supervision_Timeout;

                                device->m_machine.event &= ~BLE_EVENT_HCI_LE_CONNECTION_UPDATE_COMPLETE_EVENT;

                                if (device->m_machine.connected) {
                                    device->m_connection_parameters_callback();
                                } else {
                                    if (!device->m_machine.event) {
                                        connect(device);
                                    }
                                }
                                break;
                            }
                        }
                        break;
                    }

                    case HCI_LE_LONG_TERM_KEY_REQUEST_EVENT: {
                        // #### negative reply ? or what ?
                        break;
                    }

                    case HCI_LE_DATA_LENGTH_CHANGE_EVENT: {
                        const hci_le_data_length_change_event_rp0 *eparam = (const hci_le_data_length_change_event_rp0*)&hci_le_event->data[0];

#if (BLE_TRACE_SUPPORTED == 1)
                        armv7m_rtt_printf("DTA-LENGTH-CHANGE %d/%d, %d/%d\r\n", eparam->MaxTxOctets, eparam->MaxTxTime, eparam->MaxRxOctets, eparam->MaxRxTime);
#endif
                        break;
                    }
                        
                    case HCI_LE_READ_REMOTE_FEATURES_COMPLETE_EVENT:
                      // case HCI_LE_DATA_LENGTH_CHANGE_EVENT:
                    case HCI_LE_READ_LOCAL_P256_PUBLIC_KEY_COMPLETE_EVENT:
                    case HCI_LE_GENERATE_DHKEY_COMPLETE_EVENT:
                    case HCI_LE_PHY_UPDATE_COMPLETE_EVENT:
                        break;
                    
                    default:
#if (BLE_TRACE_SUPPORTED == 1)
                        armv7m_rtt_printf("UNEXPTECT HCI LE EVENT !!!\n");
#endif
                        break;
                    }
                    break;

                case HCI_VENDOR_SPECIFIC_EVENT:
                    hci_vs_event = (const evt_blue_aci*)&hci_event->data[0];

                    switch (hci_vs_event->ecode) {

                      /* HAL */
                    case ACI_HAL_END_OF_RADIO_ACTIVITY_EVENT:
                    case ACI_HAL_WARNING_EVENT:
                        break;
                          
                      /* GAP */
                    case ACI_GAP_PAIRING_COMPLETE_EVENT: {
                        const aci_gap_pairing_complete_event_rp0 *eparam = (const aci_gap_pairing_complete_event_rp0*)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;

                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

                                armv7m_atomic_andh(&device->m_machine.event, ~BLE_EVENT_GAP_PAIRING_COMPLETE);

                                if (eparam->Status == BLE_STATUS_SUCCESS) {
                                    if (!device->m_machine.connected && !device->m_machine.event) {
                                        connect(device);
                                    }
                                } else {
                                    armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GAP_TERMINATE);
                                }

                                break;
                            }
                        }
                        break;
                    }

                    case ACI_GAP_BOND_LOST_EVENT: {
                        const aci_gap_bond_lost_event_rp0 *eparam = (const aci_gap_bond_lost_event_rp0*)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;
                        
                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

                                if (m_gap.bonding) {
                                    armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GAP_ALLOW_REBOND);
                                } else {
                                    armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GAP_TERMINATE);
                                }

                                break;
                            }
                        }
                        break;
                    }

                    case ACI_GAP_LIMITED_DISCOVERABLE_EVENT: {
                        if (m_state == BLE_LOCAL_STATE_ADVERTISING) {
                            m_state = BLE_LOCAL_STATE_READY;

#if (BLE_TRACE_SUPPORTED == 1)
                            armv7m_rtt_printf("REQUEST-STOP\r\n");
#endif
                            
                            m_stop_callback();
                        }
                        break;
                    }
                        
                    case ACI_GAP_PROC_COMPLETE_EVENT:
                        break;

                    case ACI_GAP_ADDR_NOT_RESOLVED_EVENT:
                        // ##### ????
                        break;

                    case ACI_GAP_PASS_KEY_REQ_EVENT: {
                        const aci_gap_pass_key_req_event_rp0 *eparam = (const aci_gap_pass_key_req_event_rp0*)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;

                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

                                armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GAP_PASSKEY);
                                break;
                            }
                        }
                        break;
                    }
                      
                        
                    case ACI_GAP_AUTHORIZATION_REQ_EVENT:
                    case ACI_GAP_NUMERIC_COMPARISON_VALUE_EVENT:
                    case ACI_GAP_KEYPRESS_NOTIFICATION_EVENT:
#if (BLE_TRACE_SUPPORTED == 1)
                        armv7m_rtt_printf("REQUEST-UNEXPTECT ACI GAP EVENT !!!\n");
#endif
                        break;
                        
                        
                        /* L2CAP */
                    case ACI_L2CAP_CONNECTION_UPDATE_RESP_EVENT:
                        break;
                        
                    case ACI_L2CAP_PROC_TIMEOUT_EVENT:
                        break;

                    case ACI_L2CAP_CONNECTION_UPDATE_REQ_EVENT: {
                        const aci_l2cap_connection_update_req_event_rp0 *eparam = (const aci_l2cap_connection_update_req_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;

                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];
                              
                                device->m_machine.params.interval_min = eparam->Interval_Min;
                                device->m_machine.params.interval_max = eparam->Interval_Max;
                                device->m_machine.params.latency = eparam->Latency;
                                device->m_machine.params.timeout = eparam->Timeout_Multiplier;
                                device->m_machine.params.sequence = eparam->Identifier;
                                device->m_machine.params.accept = 1;

                                armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_L2CAP_CONNECTION_PARAMETER_CONFIRM);
                            }
                        }
                        break;
                    }
                        
                    case ACI_L2CAP_COMMAND_REJECT_EVENT:
                        break;

                        /* GATT/ATT */
                    case ACI_GATT_ATTRIBUTE_MODIFIED_EVENT: {
                        const aci_gatt_attribute_modified_event_rp0 *eparam = (const aci_gatt_attribute_modified_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        BLEServerService *service;
                        BLEServerCharacteristic *characteristic;
                        uint16_t config;
                        int index;

#if (BLE_TRACE_SUPPORTED == 1)
                        armv7m_rtt_printf("ATTRIBUTE-MODIFIED %04x/%04x/%04x\r\n", eparam->Attr_Handle, eparam->Offset, eparam->Attr_Data_Length);
#endif
                        
                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

                                for (service = m_server.children; service; service = service->m_sibling) {
                                    if ((service->m_handle < eparam->Attr_Handle) && (service->m_end_group_handle >= eparam->Attr_Handle)) {
                                        for (characteristic = service->m_children; characteristic; characteristic = characteristic->m_sibling) {
                                            if (eparam->Attr_Handle == (characteristic->m_handle + BLE_VALUE_ATTRIB_HANDLE_OFFSET)) {
                                                if (!(characteristic->m_control & BLE_SERVER_CONTROL_MODIFIED) && (!characteristic->m_write_handle || (characteristic->m_write_handle == eparam->Connection_Handle))) {
                                                    memcpy(&characteristic->m_value_data[characteristic->m_value_size + (eparam->Offset & 0x7fff)],
                                                           &eparam->Attr_Data[0],
                                                           (eparam->Attr_Data_Length - (eparam->Offset & 0x7fff)));
                                            
                                                    if (!(eparam->Offset & 0x8000)) {
                                                        /* Ok, all chunks arrived, set BLE_SERVER_CONTROL_MODIFIED and m_write_length.
                                                         * If BLE_SERVER_CONTROL_NOTIFY is not set, then set BLE_SERVER_CONTROL_WRITTEN.
                                                         * This is needed so that we can read the buffered data, which is not possible
                                                         * with a pending BLE_SERVER_CONTROL_NOTIFY.
                                                         */
                                                        armv7m_atomic_orb(&characteristic->m_control, BLE_SERVER_CONTROL_MODIFIED);
                                                        characteristic->m_write_length = eparam->Attr_Data_Length;

                                                        do {
                                                            control = characteristic->m_control;

                                                            if (control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET)) {
                                                                break;
                                                            }
                                                        } while (armv7m_atomic_casb(&characteristic->m_control, control, (control | BLE_SERVER_CONTROL_WRITTEN)) != control);

                                                        if (!(control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET))) {
#if (BLE_TRACE_SUPPORTED == 1)
                                                            armv7m_rtt_printf("ATTRIB-WRITTEN (\"%s\")\r\n", uuid_string(characteristic->m_uuid));
#endif
                                                            characteristic->m_value_length = characteristic->m_write_length;
                                                            characteristic->m_write_length = 0;
                                                    
                                                            memcpy(&characteristic->m_value_data[0], &characteristic->m_value_data[characteristic->m_value_size], characteristic->m_value_length);

                                                            characteristic->m_write_callback();
                                                        }
                                                    } else {
                                                        characteristic->m_write_handle = eparam->Connection_Handle;
                                                    }
                                                }
                                                break;
                                            }

                                            if ((characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) && (eparam->Attr_Handle == (characteristic->m_handle + BLE_CCCD_ATTRIB_HANDLE_OFFSET))) {
                                                config = characteristic->m_config;
                                                characteristic->m_config = (characteristic->m_config & ~(3 << (2 * device->m_index))) | ((eparam->Attr_Data[0] & 3) << (2 * device->m_index));

#if (BLE_TRACE_SUPPORTED == 1)
                                                armv7m_rtt_printf("ATTRIB-SUBSCRIBED %04x (\"%s\")\r\n", characteristic->m_config, uuid_string(characteristic->m_uuid));
#endif
                                                
                                                if ((config ^ characteristic->m_config) & ((characteristic->m_properties & BLE_PROPERTY_INDICATE) ? 0xaaaa : 0x5555)) {
                                                    characteristic->m_subscribe_callback();
                                                }

                                                break;
                                            }
                                        }

                                        if (characteristic) {
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        break;
                    }

                    case ACI_GATT_PROC_TIMEOUT_EVENT: {
                        const aci_gatt_proc_timeout_event_rp0 *eparam = (const aci_gatt_proc_timeout_event_rp0*)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;
                        
                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];
                                
                                armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GAP_TERMINATE);

                                break;
                            }
                        }
                        break;
                    }

                    case ACI_ATT_EXCHANGE_MTU_RESP_EVENT: {
                        const aci_att_exchange_mtu_resp_event_rp0 *eparam = (const aci_att_exchange_mtu_resp_event_rp0*)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;

                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];
                        
#if (BLE_TRACE_SUPPORTED == 1)
                                armv7m_rtt_printf("ATT_EXCHANGE_MTU %d\n", eparam->Server_RX_MTU);
#endif                        
                                device->m_mtu = eparam->Server_RX_MTU;

                                armv7m_atomic_andh(&device->m_machine.event, ~BLE_EVENT_GATT_MTU_EXCHANGE);
        
                                if (!device->m_machine.connected) {
                                    if (!device->m_machine.event) {
                                        connect(device);
                                    }
                                } else {
                                    device->m_mtu_exchange_callback();
                                }
                                break;
                            }
                        }
                        break;
                    }
                        
                    case ACI_ATT_FIND_INFO_RESP_EVENT: {
                        const aci_att_find_info_resp_event_rp0 *eparam = (const aci_att_find_info_resp_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;
                        BLEClientCharacteristic *characteristic;
                        uint16_t handle, uuid16;
                        const uint8_t *data, *data_e, *uuid_data;
                        int uuid_type;
                        
                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

                                data = &eparam->Handle_UUID_Pair[0];
                                data_e = data + eparam->Event_Data_Length;
                        
                                characteristic = device->m_machine.discovery_characteristic;

                                while (data != data_e) {
                                    handle = (data[0] << 0) | (data[1] << 8);

                                    if (eparam->Format == 1) {
                                        uuid_type = BLE_UUID_TYPE_16;
                                        uuid_data = &data[2];
                                    
                                        uuid16 = (data[2] << 0) | (data[3] << 8);

                                        if (uuid16 == 0x2902) {
                                            characteristic->m_config_handle = handle;
                                        }
                            
                                        data += 4;
                                    } else {
                                        uuid_type = BLE_UUID_TYPE_128;
                                        uuid_data = &data[2];

                                        data += 18;
                                    }
                                
#if (BLE_TRACE_SUPPORTED == 1)
                                    {
                                        BLEUuid uuid = BLEUuid(uuid_data, (BLEUuidType)uuid_type);
                                    
                                        armv7m_rtt_printf("DESCRIPTOR \"%s\", %04x (\"%s\")\n", uuid_string(uuid), handle, uuid_string(device->m_machine.discovery_characteristic->m_uuid));
                                    }
#endif                                
                                }

                                break;
                            }
                        }
                        break;
                    }
                        
                    case ACI_ATT_FIND_BY_TYPE_VALUE_RESP_EVENT: {
                        const aci_att_find_by_type_value_resp_event_rp0 *eparam = (const aci_att_find_by_type_value_resp_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;
                        BLEClientService *service, *service_current, **service_previous;
                        uint16_t handle, end_group_handle;
                        const uint8_t *data, *data_e;

                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

                                data = (const uint8_t*)&eparam->Attribute_Group_Handle_Pair[0];
                                data_e = data + (eparam->Num_of_Handle_Pair * 4);

                                while (data != data_e) {
                                    handle = (data[0] << 0) | (data[1] << 8);
                                    end_group_handle = (data[2] << 0) | (data[3] << 8);

                                    data += 4;

#if (BLE_TRACE_SUPPORTED == 1)
                                    armv7m_rtt_printf("SERVICE \"%s\", %04x/%04x\n", uuid_string(device->m_machine.discovery_uuid), handle, end_group_handle);
#endif
                                    
                                    for (service = nullptr, service_previous = &device->m_client.children, service_current = *service_previous; service_current; service_previous = &service_current->m_sibling, service_current = *service_previous) {
                                        if (handle == service_current->m_handle) {
                                            service = service_current;
                                            break;
                                        }
                                    
                                        if (end_group_handle < service_current->m_handle) {
                                            break;
                                        }
                                    }

                                    if (!service) {
                                        service = new BLEClientService(device->m_machine.discovery_uuid);
                                    
                                        if (service) {
                                            service->m_handle = handle;
                                            service->m_end_group_handle = end_group_handle;
                                            service->m_parent = device;
                                            service->m_sibling = service_current;
                                        
                                            *service_previous = service;
                                        
                                            device->m_client.count++;
                                        }
                                    }

                                    /* Setup characteristic discovery
                                     */
                                    device->m_machine.discovery_service = service;

                                    break;
                                }

                                break;
                            }
                        }
                        break;
                    }
                      
                    case ACI_ATT_READ_BY_TYPE_RESP_EVENT: {
                        const aci_att_read_by_type_resp_event_rp0 *eparam = (const aci_att_read_by_type_resp_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;
                        BLEClientService *service;
                        BLEClientCharacteristic *characteristic;
                        uint8_t properties;
                        uint16_t handle, value_handle;
                        const uint8_t *data, *data_e, *uuid_data;
                        int uuid_type;
                        BLEUuid uuid;
                    
                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

                                service = device->m_machine.discovery_service; 
                                
                                data = &eparam->Handle_Value_Pair_Data[0];
                                data_e = data + eparam->Data_Length;

                                while (data != data_e) {
                                    
                                    handle = (data[0] << 0) | (data[1] << 8);
                                    properties = data[2];
                                    value_handle = (data[3] << 0) | (data[4] << 8);

                                    if (eparam->Handle_Value_Pair_Length == 7) {
                                        uuid_type = BLE_UUID_TYPE_16;
                                        uuid_data = &data[5];

                                        data += 7;
                                    } else {
                                        uuid_type = BLE_UUID_TYPE_128;
                                        uuid_data = &data[5];

                                        data += 21;
                                    }

                                    uuid = BLEUuid(uuid_data, (BLEUuidType)uuid_type);
                                
#if (BLE_TRACE_SUPPORTED == 1)
                                    armv7m_rtt_printf("CHARACTERISTIC \"%s\", %02x, %04x/%04x (\"%s\")\n", uuid_string(uuid), properties, handle, value_handle, uuid_string(device->m_machine.discovery_service->m_uuid));
#endif
                                
                                    characteristic = new BLEClientCharacteristic(uuid, properties);

                                    if (characteristic) {
                                        characteristic->m_handle = handle;
                                        characteristic->m_value_handle = value_handle;
                                        characteristic->m_parent = service;
                                    
                                        if (device->m_machine.discovery_characteristic) {
                                            device->m_machine.discovery_characteristic->m_sibling = characteristic;
                                        } else {
                                            service->m_children = characteristic;
                                        }

                                        device->m_machine.discovery_characteristic = characteristic;
                                        
                                        service->m_count++;
                                    }
                                }

                                break;
                            }
                        }
                        break;
                    }
                        
                    case ACI_ATT_READ_RESP_EVENT: {
                        const aci_att_read_resp_event_rp0 *eparam = (const aci_att_read_resp_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;
                        uint32_t count;

                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];
                                
                                count = eparam->Event_Data_Length;
                                
                                if ((device->m_machine.read_offset + count) > BLE_MAX_ATT_SIZE) {
                                    count = BLE_MAX_ATT_SIZE - device->m_machine.read_offset;
                                }
                                
                                memcpy(&device->m_machine.read_data[device->m_machine.read_offset], &eparam->Attribute_Value[0], count);
                                
                                device->m_machine.read_offset += count;
                                
                                armv7m_rtt_printf("READ_RESP %04x, %d (%d/%d)\n", eparam->Connection_Handle, eparam->Event_Data_Length, count, device->m_machine.read_offset);

                                break;
                            }
                        }
                        break;
                    }
                      
                    case ACI_ATT_READ_BLOB_RESP_EVENT: {
                        const aci_att_read_blob_resp_event_rp0 *eparam = (const aci_att_read_blob_resp_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;
                        uint32_t count;

                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

                                count = eparam->Event_Data_Length;
                        
                                if ((device->m_machine.read_offset + count) > BLE_MAX_ATT_SIZE) {
                                    count = BLE_MAX_ATT_SIZE - device->m_machine.read_offset;
                                }
                                
                                memcpy(&device->m_machine.read_data[device->m_machine.read_offset], &eparam->Attribute_Value[0], count);
                                
                                device->m_machine.read_offset += count;
                                
                                armv7m_rtt_printf("READ_BLOAB_RESP %04x, %d (%d/%d)\n", eparam->Connection_Handle, eparam->Event_Data_Length, count, device->m_machine.read_offset);

                                break;
                            }
                        }
                        break;
                    }
                      
                    case ACI_ATT_READ_MULTIPLE_RESP_EVENT:
                        break;
                        
                    case ACI_ATT_READ_BY_GROUP_TYPE_RESP_EVENT: {
                        const aci_att_read_by_group_type_resp_event_rp0 *eparam = (const aci_att_read_by_group_type_resp_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;
                        BLEClientService *service, *service_current, **service_previous;
                        uint16_t handle, end_group_handle;
                        const uint8_t *data, *data_e, *uuid_data;
                        int uuid_type;
                        BLEUuid uuid;
                        
                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

                                data = &eparam->Attribute_Data_List[0];
                                data_e = data + eparam->Data_Length;

                                while (data != data_e) {
                                    handle = (data[0] << 0) | (data[1] << 8);
                                    end_group_handle = (data[2] << 0) | (data[3] << 8);

                                    if (eparam->Attribute_Data_Length == 6) {
                                        uuid_type = BLE_UUID_TYPE_16;
                                        uuid_data = &data[4];

                                        data += 6;
                                    } else {
                                        uuid_type = BLE_UUID_TYPE_128;
                                        uuid_data = &data[4];

                                        data += 20;
                                    }

                                    uuid = BLEUuid(uuid_data, (BLEUuidType)uuid_type);

#if (BLE_TRACE_SUPPORTED == 1)
                                    armv7m_rtt_printf("SERVICE \"%s\", %04x/%04x\n", uuid_string(uuid), handle, end_group_handle);
#endif
                                    
                                    for (service = nullptr, service_previous = &device->m_client.children, service_current = *service_previous; service_current; service_previous = &service_current->m_sibling, service_current = *service_previous) {
                                        if (handle == service_current->m_handle) {
                                            service = service_current;;
                                            break;
                                        }
                                    
                                        if (end_group_handle < service_current->m_handle) {
                                            break;
                                        }
                                    }
                                
                                    if (!service) {
                                        service = new BLEClientService(uuid);
                                    
                                        if (service) {
                                            service->m_handle = handle;
                                            service->m_end_group_handle = end_group_handle;
                                            service->m_parent = device;
                                            service->m_sibling = service_current;
                                        
                                            *service_previous = service;
                                        
                                            device->m_client.count++;
                                        }
                                    }
                                }

                                break;
                            }
                        }
                        break;
                    }
                        
                    case ACI_ATT_PREPARE_WRITE_RESP_EVENT:
                        break;
                      
                    case ACI_ATT_EXEC_WRITE_RESP_EVENT:
                        break;

                    case ACI_GATT_INDICATION_EVENT:
                    case ACI_GATT_NOTIFICATION_EVENT: {
                        const aci_gatt_notification_event_rp0 *eparam = (const aci_gatt_notification_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;
                        BLEClientService *service;
                        BLEClientCharacteristic *characteristic;
                        uint8_t control;
                                                
                        armv7m_rtt_printf(((hci_vs_event->ecode == ACI_GATT_INDICATION_EVENT) ? "INDICATION %04x/%04x\n" :  "NOTIFICATION %04x/%04x\n"), eparam->Connection_Handle, eparam->Attribute_Handle);

                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];
                        
                                for (service = device->m_client.children; service; service = service->m_sibling) {
                                    if ((service->m_handle < eparam->Attribute_Handle) && (service->m_end_group_handle >= eparam->Attribute_Handle)) {
                                        for (characteristic = service->m_children; characteristic; characteristic = characteristic->m_sibling) {
                                            if (eparam->Attribute_Handle == characteristic->m_value_handle) {
                                                if (!(characteristic->m_control & BLE_CLIENT_CONTROL_MODIFIED)) {
                                                    characteristic->m_write_length = 0;

                                                    if (characteristic->m_write_size < eparam->Attribute_Value_Length) {
                                                        characteristic->m_write_data = (uint8_t*)realloc(characteristic->m_write_data, eparam->Attribute_Value_Length);
                                                
                                                        if (characteristic->m_write_data) {
                                                            characteristic->m_write_size = eparam->Attribute_Value_Length;
                                                        } else {
                                                            characteristic->m_write_size = 0;
                                                        }
                                                    }
                                            
                                                    if (characteristic->m_write_size) {
                                                        memcpy(&characteristic->m_write_data[0], &eparam->Attribute_Value[0], eparam->Attribute_Value_Length);

                                                        characteristic->m_write_length = eparam->Attribute_Value_Length;
                                                    }

                                                    if (hci_vs_event->ecode == ACI_GATT_INDICATION_EVENT) {
                                                        armv7m_atomic_orb(&characteristic->m_control, (BLE_CLIENT_CONTROL_MODIFIED | BLE_CLIENT_CONTROL_CONFIRM));
                                                    } else {
                                                        armv7m_atomic_orb(&characteristic->m_control, (BLE_CLIENT_CONTROL_MODIFIED));
                                                    }

                                                    do {
                                                        control = characteristic->m_control;

                                                        if (control & (BLE_CLIENT_CONTROL_READ | BLE_CLIENT_CONTROL_WRITE)) {
                                                            break;
                                                        }
                                                    } while (armv7m_atomic_casb(&characteristic->m_control, control, (control | BLE_CLIENT_CONTROL_WRITTEN)) != control);

                                                    if (!(control & (BLE_CLIENT_CONTROL_READ | BLE_CLIENT_CONTROL_WRITE))) {
#if (BLE_TRACE_SUPPORTED == 1)
                                                        armv7m_rtt_printf("ATTRIB-NOTIFY (\"%s\")\r\n", uuid_string(characteristic->m_uuid));
#endif

                                                        characteristic->m_value_length = 0;

                                                        if (characteristic->m_value_size < characteristic->m_write_length) {
                                                            characteristic->m_value_data = (uint8_t*)realloc(characteristic->m_value_data, characteristic->m_write_length);
                                                    
                                                            if (characteristic->m_value_data) {
                                                                characteristic->m_value_size = characteristic->m_write_length;
                                                            } else {
                                                                characteristic->m_value_size = 0;
                                                            }
                                                        }
                                                
                                                        if (characteristic->m_value_size) {
                                                            memcpy(&characteristic->m_value_data[0], &characteristic->m_write_data[0], characteristic->m_write_length);
                                                    
                                                            characteristic->m_value_length = characteristic->m_write_length;
                                                        }
                                                
                                                        characteristic->m_notify_callback();
                                                    }
                                                }
                                                break;
                                            }
                                        }

                                        if (characteristic) {
                                            break;
                                        }
                                    }
                                }

                                break;
                            }
                        }
                        break;
                    }
                      
                    case ACI_GATT_PROC_COMPLETE_EVENT: {
                        const aci_gatt_proc_complete_event_rp0 *eparam = (const aci_gatt_proc_complete_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;
                        BLEClientService *service;
                        BLEClientCharacteristic *characteristic;
                        Callback callback;
                        
                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

                                device->m_machine.event &= ~BLE_EVENT_GATT_PROC_COMPLETE;
                          
                                switch (device->m_machine.procedure) {
                                case BLE_GATT_PROCEDURE_NONE:
                                    break;

                                case BLE_GATT_PROCEDURE_DISCOVER_SERVICE_BY_UUID:
                                    switch (device->m_machine.discovery_request) {
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE:
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_NOT_READY:
                                        break;

                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER:
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_SERVICES:
                                        break;
                                    
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_UUID:
                                        if (device->m_machine.discovery_service) {
                                            service = device->m_machine.discovery_service;

                                            if (((service->m_handle +1) != service->m_end_group_handle) && !service->m_children) {
                                                device->m_machine.discovery_characteristic = nullptr;
                                        
                                                armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GATT_DISCOVER_CHARACTERISTICS);
                                            } else {
                                                device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;
                                                device->m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;
                                        
                                                device->m_done_callback();
                                            }
                                        } else {
                                            device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;
                                            device->m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;
                                        
                                            device->m_done_callback();
                                        }
                                        break;

                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_SERVICE:
                                    default:
                                        break;
                                    }
                                    break;
                                
                                case BLE_GATT_PROCEDURE_DISCOVER_SERVICES:
                                    switch (device->m_machine.discovery_request) {
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE:
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_NOT_READY:
                                        break;

                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER:
                                        /* Service discovery done, move on to discover the characteristics of each service.
                                         */
                                    
                                        device->m_client.discovered = true;
                                    
                                        for (service = device->m_client.children; service; service = service->m_sibling) {
                                            if (((service->m_handle +1) != service->m_end_group_handle) && !service->m_children) {
                                                break;
                                            }
                                        }
                                    
                                        if (service) {
                                            device->m_machine.discovery_service = service;
                                            device->m_machine.discovery_characteristic = nullptr;
                                        
                                            armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GATT_DISCOVER_CHARACTERISTICS);
                                        } else {
                                            device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;
                                            device->m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;
                                        
                                            device->m_done_callback();
                                        }
                                        break;
                                    
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_SERVICES:
                                        device->m_client.discovered = true;
                                    
                                        device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;
                                        device->m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;
                                    
                                        device->m_done_callback();
                                        break;

                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_UUID:
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_SERVICE:
                                        break;
                                    }
                                    break;
                                
                                case BLE_GATT_PROCEDURE_DISCOVER_CHARACTERISTICS:
                                    switch (device->m_machine.discovery_request) {
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE:
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_NOT_READY:
                                        break;

                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER:
                                        /* First check whether another service needs it's characterics to be discovered.
                                         */

                                        for (service = device->m_machine.discovery_service->m_sibling; service; service = service->m_sibling) {
                                            if (((service->m_handle +1) != service->m_end_group_handle) && !service->m_children) {
                                                break;
                                            }
                                        }
                                    
                                        if (service) {
                                            device->m_machine.discovery_service = service;
                                            device->m_machine.discovery_characteristic = nullptr;

                                            armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GATT_DISCOVER_CHARACTERISTICS);
                                        } else {
                                            /* If all services had their characteristics discovered, move on to discover the descriptors.
                                             */

                                            for (service = device->m_client.children, characteristic = nullptr; service; service = service->m_sibling) {
                                                for (characteristic = service->m_children; characteristic; characteristic = characteristic->m_sibling) {
                                                    if (characteristic->m_value_handle != (characteristic->m_sibling ? (characteristic->m_sibling->m_handle -1) : characteristic->m_parent->m_end_group_handle)) {
                                                        if ((characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) &&
                                                            ((characteristic->m_value_handle +1) == (characteristic->m_sibling ? (characteristic->m_sibling->m_handle -1) : characteristic->m_parent->m_end_group_handle))) {
                                                            characteristic->m_config_handle = characteristic->m_value_handle +1;
                                                        } else {
                                                            break;
                                                        }
                                                    }
                                                }

                                                if (characteristic) {
                                                    break;
                                                }
                                            }

                                            if (characteristic) {
                                                device->m_machine.discovery_service = service;
                                                device->m_machine.discovery_characteristic = characteristic;

                                                armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GATT_DISCOVER_DESCRIPTORS);
                                            } else {
                                                device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;
                                                device->m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;

                                                device->m_done_callback();
                                            }
                                        }
                                        break;
                                    
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_SERVICES:
                                        break;
                                
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_UUID:
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_SERVICE:
                                        /* All characteristics of this service have been discovered, move on to the descriptors for
                                         * each characteristic of the service.
                                         */

                                        service = device->m_machine.discovery_service;

                                        for (characteristic = service->m_children; characteristic; characteristic = characteristic->m_sibling) {
                                            if (characteristic->m_value_handle != (characteristic->m_sibling ? (characteristic->m_sibling->m_handle -1) : characteristic->m_parent->m_end_group_handle)) {
                                                if ((characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) &&
                                                    ((characteristic->m_value_handle +1) == (characteristic->m_sibling ? (characteristic->m_sibling->m_handle -1) : characteristic->m_parent->m_end_group_handle))) {
                                                    characteristic->m_config_handle = characteristic->m_value_handle +1;
                                                } else {
                                                    break;
                                                }
                                            }
                                        }

                                        if (characteristic) {
                                            device->m_machine.discovery_characteristic = characteristic;

                                            armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GATT_DISCOVER_DESCRIPTORS);
                                        } else {
                                            device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;
                                            device->m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;

                                            device->m_done_callback();
                                        }
                                        break;
                                    }
                                    break;

                                case BLE_GATT_PROCEDURE_DISCOVER_DESCRIPTORS:
                                    switch (device->m_machine.discovery_request) {
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE:
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_NOT_READY:
                                        break;

                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_SERVICES:
                                        break;
                                    
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER:
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_UUID:
                                    case BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER_CHARACTERISTICS_BY_SERVICE:
                                        service = device->m_machine.discovery_service;

                                        for (characteristic = device->m_machine.discovery_characteristic->m_sibling; characteristic; characteristic = characteristic->m_sibling) {
                                            if (characteristic->m_value_handle != (characteristic->m_sibling ? (characteristic->m_sibling->m_handle -1) : characteristic->m_parent->m_end_group_handle)) {
                                                if ((characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) &&
                                                    ((characteristic->m_value_handle +1) == (characteristic->m_sibling ? (characteristic->m_sibling->m_handle -1) : characteristic->m_parent->m_end_group_handle))) {
                                                    characteristic->m_config_handle = characteristic->m_value_handle +1;
                                                } else {
                                                    break;
                                                }
                                            }
                                        }

                                        if (!characteristic && (device->m_machine.discovery_request == BLE_PEER_DEVICE_DISCOVERY_REQUEST_DISCOVER)) {
                                            for (service = service->m_sibling; service; service = service->m_sibling) {
                                                for (characteristic = service->m_children; characteristic; characteristic = characteristic->m_sibling) {
                                                    if (characteristic->m_value_handle != (characteristic->m_sibling ? (characteristic->m_sibling->m_handle -1) : characteristic->m_parent->m_end_group_handle)) {
                                                        if ((characteristic->m_properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) &&
                                                            ((characteristic->m_value_handle +1) == (characteristic->m_sibling ? (characteristic->m_sibling->m_handle -1) : characteristic->m_parent->m_end_group_handle))) {
                                                            characteristic->m_config_handle = characteristic->m_value_handle +1;
                                                        } else {
                                                            break;
                                                        }
                                                    }
                                                }
                                            
                                                if (characteristic) {
                                                    break;
                                                }
                                            }
                                        }
                                    
                                        if (characteristic) {
                                            device->m_machine.discovery_service = service;
                                            device->m_machine.discovery_characteristic = characteristic;
                                        
                                            armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GATT_DISCOVER_DESCRIPTORS);
                                        } else {
                                            device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;
                                            device->m_machine.discovery_request = BLE_PEER_DEVICE_DISCOVERY_REQUEST_NONE;
                                        
                                            device->m_done_callback();
                                        }
                                        break;
                                    }
                                    break;

                                case BLE_GATT_PROCEDURE_READ_VALUE:
                                    if (device->m_machine.read_offset == (device->m_mtu -1)) {
                                        armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GATT_READ_VALUE_LONG);
                                        break;
                                    }

                                    [[fallthrough]];
                                    /* FALLTHROU */

                                case BLE_GATT_PROCEDURE_READ_VALUE_LONG:
                                    characteristic = device->m_machine.read_current;

                                    device->m_machine.read_current = nullptr;
                                    device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;

                                    callback = characteristic->m_done_callback;
                                

                                    characteristic->m_value_length = 0;
                                
                                    if (characteristic->m_value_size < device->m_machine.read_offset) {
                                        characteristic->m_value_data = (uint8_t*)realloc(characteristic->m_value_data, device->m_machine.read_offset);
                                    
                                        if (characteristic->m_value_data) {
                                            characteristic->m_value_size = device->m_machine.read_offset;
                                        } else {
                                            characteristic->m_value_size = 0;
                                        }
                                    }
                                
                                    if (characteristic->m_value_size) {
                                        memcpy(&characteristic->m_value_data[0], &device->m_machine.read_data[0], device->m_machine.read_offset);
                                    
                                        characteristic->m_value_length = device->m_machine.read_offset;

                                        armv7m_atomic_andb(&characteristic->m_control, ~BLE_CLIENT_CONTROL_SEQUENCE);
                                    
                                        characteristic->m_status = BLE_STATUS_SUCCESS;
                                    } else {
                                        characteristic->m_value_length = 0;
                                    
                                        characteristic->m_status = BLE_STATUS_FAILURE;
                                    }

                                    control = armv7m_atomic_andb(&characteristic->m_control, ~BLE_CLIENT_CONTROL_READ);                        
                                
                                    callback();

                                    if (device->m_machine.request_head) {
                                        device->request();
                                    }
                                    break;

                                case BLE_GATT_PROCEDURE_WRITE_VALUE:
                                case BLE_GATT_PROCEDURE_WRITE_VALUE_LONG:
                                case BLE_GATT_PROCEDURE_WRITE_VALUE_WITHOUT_RESPONSE:
                                    characteristic = device->m_machine.write_current;

                                    device->m_machine.write_current = nullptr;
                                    device->m_machine.procedure = BLE_GATT_PROCEDURE_NONE;

                                    callback = characteristic->m_done_callback;
                                
                                    characteristic->m_status = BLE_STATUS_SUCCESS;

                                    control = armv7m_atomic_andb(&characteristic->m_control, ~(BLE_CLIENT_CONTROL_WRITE | BLE_CLIENT_CONTROL_WRITE_WITHOUT_RESPONSE));                        

                                    if ((control & (BLE_CLIENT_CONTROL_MODIFIED | BLE_CLIENT_CONTROL_WRITTEN)) == BLE_CLIENT_CONTROL_MODIFIED) {
                                        do {
                                            control = characteristic->m_control;
                                        
                                            if (control & (BLE_CLIENT_CONTROL_READ | BLE_CLIENT_CONTROL_WRITE)) {
                                                break;
                                            }
                                        } while (armv7m_atomic_casb(&characteristic->m_control, control, (control | BLE_CLIENT_CONTROL_WRITTEN)) != control);

                                        if (!(control & (BLE_CLIENT_CONTROL_READ | BLE_CLIENT_CONTROL_WRITE))) {
#if (BLE_TRACE_SUPPORTED == 1)
                                            armv7m_rtt_printf("ATTRIB-NOTIFY (\"%s\")\r\n", uuid_string(characteristic->m_uuid));
#endif

                                            characteristic->m_value_length = 0;

                                            if (characteristic->m_value_size < characteristic->m_write_length) {
                                                characteristic->m_value_data = (uint8_t*)realloc(characteristic->m_value_data, characteristic->m_write_length);
                                            
                                                if (characteristic->m_value_data) {
                                                    characteristic->m_value_size = characteristic->m_write_length;
                                                } else {
                                                    characteristic->m_value_size = 0;
                                                }
                                            }
                                        
                                            if (characteristic->m_value_size) {
                                                memcpy(&characteristic->m_value_data[0], &characteristic->m_write_data[0], characteristic->m_write_length);
                                            
                                                characteristic->m_value_length = characteristic->m_write_length;
                                            }
                                        
                                            characteristic->m_notify_callback();
                                        }
                                    }
                                
                                    callback();

                                    if (device->m_machine.request_head) {
                                        device->request();
                                    }
                                    break;

                                case BLE_GATT_PROCEDURE_WRITE_CONFIG:
                                    device->m_machine.config_current = nullptr;

                                    if (device->m_machine.config_request) {
                                        device->request();
                                    }
                                    break;
                                
                                default:
                                    break;
                                }

                                break;
                            }
                        }
                        break;
                    }
                        
                    case ACI_GATT_ERROR_RESP_EVENT: {
                        const aci_gatt_error_resp_event_rp0 *eparam = (const aci_gatt_error_resp_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;

                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

#if 1
                                armv7m_rtt_printf("ERROR %02x, %04x == %04x (%d / %d)\n", eparam->Error_Code, eparam->Connection_Handle, device->m_handle, device->m_machine.procedure, device->m_machine.discovery_request);
#endif
                        
                                if (eparam->Error_Code == 0x0a) {
                                    if (device && (device->m_handle == eparam->Connection_Handle)) {
                                        switch (device->m_machine.procedure) {
                                        case BLE_GATT_PROCEDURE_NONE:
                                            break;
                                
                                        case BLE_GATT_PROCEDURE_DISCOVER_SERVICES:
                                        case BLE_GATT_PROCEDURE_DISCOVER_CHARACTERISTICS:
                                        case BLE_GATT_PROCEDURE_DISCOVER_DESCRIPTORS:
                                            break;
                                        }
                                    }
                                }

                                break;
                            }
                        }
                        break;
                    }
                      
                    case ACI_GATT_DISC_READ_CHAR_BY_UUID_RESP_EVENT:
                        break;
                        
                    case ACI_GATT_WRITE_PERMIT_REQ_EVENT:
                        break;
                        
                    case ACI_GATT_READ_PERMIT_REQ_EVENT: {
                        const aci_gatt_read_permit_req_event_rp0 *eparam = (const aci_gatt_read_permit_req_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;
                        BLEServerService *service;
                        BLEServerCharacteristic *characteristic;

                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

                                for (service = m_server.children; service; service = service->m_sibling) {
                                    if ((service->m_handle < eparam->Attribute_Handle) && (service->m_end_group_handle >= eparam->Attribute_Handle)) {
                                        for (characteristic = service->m_children; characteristic; characteristic = characteristic->m_sibling) {
                                            if (eparam->Attribute_Handle == (characteristic->m_handle + BLE_VALUE_ATTRIB_HANDLE_OFFSET)) {
#if (BLE_TRACE_SUPPORTED == 1)
                                                armv7m_rtt_printf("ATTRIB-READ (\"%s\")\r\n", uuid_string(characteristic->m_uuid));
#endif
                                                characteristic->m_read_callback();
                                                break;
                                            }
                                        }

                                        if (characteristic) {
                                            break;
                                        }
                                    }
                                }

                                armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GATT_ALLOW_READ);

                                break;
                            }
                        }
                        break;
                    }
                        
                    case ACI_GATT_READ_MULTI_PERMIT_REQ_EVENT: {
                        const aci_gatt_read_multi_permit_req_event_rp0 *eparam = (const aci_gatt_read_multi_permit_req_event_rp0 *)&hci_vs_event->data[0];
                        BLEPeerDevice *device;
                        int index;
                        BLEServerService *service;
                        BLEServerCharacteristic *characteristic;
                        uint32_t entry;
                        
                        for (index = 0; index < BLE_NUM_LINK; index++) {
                            if (m_machine.peer[index] && (m_machine.peer[index]->m_handle == eparam->Connection_Handle)) {
                                device = m_machine.peer[index];

                                for (entry = 0; entry < eparam->Number_of_Handles; entry++) {
                                    for (service = m_server.children; service; service = service->m_sibling) {
                                        if ((service->m_handle < eparam->Handle_Item[entry].Handle) && (service->m_end_group_handle >= eparam->Handle_Item[entry].Handle)) {
                                            for (characteristic = service->m_children; characteristic; characteristic = characteristic->m_sibling) {
                                                if (eparam->Handle_Item[entry].Handle == (characteristic->m_handle + BLE_VALUE_ATTRIB_HANDLE_OFFSET)) {
#if (BLE_TRACE_SUPPORTED == 1)
                                                    armv7m_rtt_printf("ATTRIB-READ (\"%s\")\r\n", uuid_string(characteristic->m_uuid));
#endif
                                                    characteristic->m_read_callback();
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                }

                                armv7m_atomic_or(&device->m_machine.request, BLE_REQUEST_GATT_ALLOW_READ);

                                break;
                            }
                        }
                        break;
                    }
                        
                    case ACI_GATT_TX_POOL_AVAILABLE_EVENT: {
                        armv7m_atomic_andh(&m_machine.event, ~BLE_EVENT_GATT_TX_POOL_AVAILABLE);
                        break;
                    }
                        
                    case ACI_GATT_SERVER_CONFIRMATION_EVENT: {
                        if (m_machine.event & BLE_EVENT_GATT_SERVER_CONFIRMATION) {
                            BLEServerCharacteristic *notify;

                            armv7m_atomic_andh(&m_machine.event, ~BLE_EVENT_GATT_SERVER_CONFIRMATION);

                            notify = m_machine.notify_current;
                            
                            m_machine.notify_current = nullptr;

#if (BLE_TRACE_SUPPORTED == 1)
                            armv7m_rtt_printf("ATTRIB-NOTIFY-DONE %02x (\"%s\", INDICATION)\r\n", BLE_STATUS_SUCCESS, uuid_string(notify->m_uuid));
#endif

                            callback = notify->m_done_callback;
                            
                            notify->m_status = BLE_STATUS_SUCCESS;

                            control = armv7m_atomic_andb(&notify->m_control, ~BLE_SERVER_CONTROL_NOTIFY);
                        
                            if ((control & (BLE_SERVER_CONTROL_MODIFIED | BLE_SERVER_CONTROL_WRITTEN)) == BLE_SERVER_CONTROL_MODIFIED) {
                                do {
                                    control = notify->m_control;
                                    
                                    if (control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET)) {
                                        break;
                                    }
                                } while (armv7m_atomic_casb(&notify->m_control, control, (control | BLE_SERVER_CONTROL_WRITTEN)) != control);
                                
                                if (!(control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET))) {
#if (BLE_TRACE_SUPPORTED == 1)
                                    armv7m_rtt_printf("ATTRIB-WRITTEN (\"%s\")\r\n", uuid_string(notify->m_uuid));
#endif
                                    notify->m_value_length = notify->m_write_length;
                                    notify->m_write_length = 0;
                                    
                                    memcpy(&notify->m_value_data[0], &notify->m_value_data[notify->m_value_size], notify->m_value_length);
                                    
                                    notify->m_write_callback();
                                }
                            }

                            callback();
                        }
                        break;
                    }
                        
                    case ACI_GATT_PREPARE_WRITE_PERMIT_REQ_EVENT:
                        break;
                        
                    case ACI_GATT_EATT_BEARER_EVENT:
                        break;

                    case ACI_GATT_MULT_NOTIFICATION_EVENT:
                        break;
                        
                    case ACI_GATT_NOTIFICATION_COMPLETE_EVENT:
                        if (m_machine.event & BLE_EVENT_GATT_NOTIFICATION_COMPLETE) {
                            BLEServerCharacteristic *notify;

                            armv7m_atomic_andh(&m_machine.event, ~BLE_EVENT_GATT_NOTIFICATION_COMPLETE);

                            notify = m_machine.notify_current;
                            
                            m_machine.notify_current = nullptr;

#if (BLE_TRACE_SUPPORTED == 1)
                            armv7m_rtt_printf("ATTRIB-NOTIFY-DONE %02x (\"%s\", NOTIFICATION)\r\n", BLE_STATUS_SUCCESS, uuid_string(notify->m_uuid));
#endif

                            callback = notify->m_done_callback;
                            
                            notify->m_status = BLE_STATUS_SUCCESS;

                            control = armv7m_atomic_andb(&notify->m_control, ~BLE_SERVER_CONTROL_NOTIFY);
                        
                            if ((control & (BLE_SERVER_CONTROL_MODIFIED | BLE_SERVER_CONTROL_WRITTEN)) == BLE_SERVER_CONTROL_MODIFIED) {
                                do {
                                    control = notify->m_control;
                                    
                                    if (control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET)) {
                                        break;
                                    }
                                } while (armv7m_atomic_casb(&notify->m_control, control, (control | BLE_SERVER_CONTROL_WRITTEN)) != control);
                                
                                if (!(control & (BLE_SERVER_CONTROL_NOTIFY | BLE_SERVER_CONTROL_SET))) {
#if (BLE_TRACE_SUPPORTED == 1)
                                    armv7m_rtt_printf("ATTRIB-WRITTEN (\"%s\")\r\n", uuid_string(notify->m_uuid));
#endif
                                    notify->m_value_length = notify->m_write_length;
                                    notify->m_write_length = 0;
                                    
                                    memcpy(&notify->m_value_data[0], &notify->m_value_data[notify->m_value_size], notify->m_value_length);
                                    
                                    notify->m_write_callback();
                                }
                            }

                            callback();
                        }
                        break;
                        
                    case ACI_GATT_READ_EXT_EVENT:
                        break;
                      
                    case ACI_GATT_INDICATION_EXT_EVENT:
                        break;
                        
                    case ACI_GATT_NOTIFICATION_EXT_EVENT:
                        break;

                    default:
                        break;
                    }
                    break;

                case HCI_COMMAND_COMPLETE_EVENT:
                case HCI_COMMAND_STATUS_EVENT:
                default:
#if (BLE_TRACE_SUPPORTED == 1)
                        armv7m_rtt_printf("REQUEST-UNEXPTECT HCI EVENT !!!\n");
#endif
                    break;
                }
            }
            
            event = (const hci_uart_pckt*)stm32wb_ipcc_ble_event();
        } while (event);
    }
    
#if (BLE_TRACE_SUPPORTED_EXT == 1)
    armv7m_rtt_printf("EVENT-LEAVE %02x/%08x/%04x\r\n", m_machine.busy, m_machine.request, m_machine.event);
#endif

    request();
}

void BLELocalDevice::request() {
    BLEServerService *service;
    BLEServerCharacteristic *characteristic;
    uint8_t control;
    
    if (m_state < BLE_LOCAL_STATE_READY) {
        return;
    }

#if (BLE_TRACE_SUPPORTED_EXT == 1)
    armv7m_rtt_printf("REQUEST-ENTER %02x/%08x/%04x\r\n", m_machine.busy, m_machine.request, m_machine.event);
#endif

    if (m_server.active) {
        BLEServerCharacteristic *notify, *request_submit, *request_next, *request_head, *request_tail;

        if (!m_machine.set_current && m_machine.set_request) {
            m_machine.set_request = false;

            for (service = m_server.children; service; service = service->m_sibling) {
                for (characteristic = service->m_children; characteristic; characteristic = characteristic->m_sibling) {
                    if (characteristic->m_control & BLE_SERVER_CONTROL_SET) {
                        m_machine.set_request = true;

                        m_machine.set_current = characteristic;
                        m_machine.set_offset = 0;
                        m_machine.set_count = 0;

                        do {
                            armv7m_atomic_orb(&characteristic->m_control, BLE_SERVER_CONTROL_SEQUENCE);
                            armv7m_atomic_andb(&characteristic->m_control, ~BLE_SERVER_CONTROL_CHANGED);
                            
                            m_machine.set_length = characteristic->m_value_length;
                            
                            memcpy(m_machine.set_data, characteristic->m_value_data, m_machine.set_length);

                            control = characteristic->m_control;
                        } while (!(control & BLE_SERVER_CONTROL_SEQUENCE) || (control & BLE_SERVER_CONTROL_CHANGED));

                        armv7m_atomic_andb(&characteristic->m_control, ~BLE_SERVER_CONTROL_SEQUENCE);

                        armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_SET_VALUE);
                        break;
                    }
                }
                
                if (m_machine.set_current) {
                    break;
                }
            }
        }
        
        if (m_machine.request_submit) {
            request_submit = (class BLEServerCharacteristic *)armv7m_atomic_swap((volatile uint32_t*)&m_machine.request_submit, (uint32_t)nullptr);

            for (request_head = nullptr, request_tail = request_submit; request_submit != nullptr; request_submit = request_next)
            {
                request_next = request_submit->m_request;

                request_submit->m_request = request_head;

                request_head = request_submit;
            }
        
            if (!m_machine.request_head)
            {
                m_machine.request_head = request_head;
            }
            else
            {
                m_machine.request_tail->m_request = request_head;
            }
        
            m_machine.request_tail = request_tail;
        }

        if (!m_machine.notify_current && m_machine.request_head) {
            notify = m_machine.request_head;

            /* There is a chance that BLE_SERVER_CONTROL_SET was set before 
             * BLE_SERVER_CONTROL_SET replaced that. Hence do not allow the
             * scheduling of a notify that is identical to set_current.
             */
            if (notify != m_machine.set_current) {
                if (m_machine.request_head == m_machine.request_tail)
                {
                    m_machine.request_head = nullptr;
                    m_machine.request_tail = nullptr;
                }
                else
                {
                    m_machine.request_head = notify->m_request;
                }
                
                m_machine.notify_current = notify;

                if (notify->m_properties & BLE_PROPERTY_INDICATE) {
                    m_machine.notify_type = (notify->m_config & 0xaaaa) ? 0x02 : 0x00;
                } else {
                    m_machine.notify_type = (notify->m_config & 0x5555) ? 0x01 : 0x00;
                }
                
                m_machine.notify_offset = 0;
                m_machine.notify_count = 0;
                m_machine.notify_length = notify->m_value_length;
                
                armv7m_atomic_or(&m_machine.request, BLE_REQUEST_GATT_NOTIFY_VALUE);
            }
        }
    }

#if (BLE_TRACE_SUPPORTED_EXT == 1)
    armv7m_rtt_printf("REQUEST-LEAVE %02x/%08x/%04x\r\n", m_machine.busy, m_machine.request, m_machine.event);
#endif

    process();
}

void BLELocalDevice::process() {
    BLEPeerDevice *device;
    int index;
    uint32_t mask, device_mask;
    uint16_t event, device_event;
    
#if (BLE_TRACE_SUPPORTED_EXT == 1)
    armv7m_rtt_printf("PROCESS-ENTER %02x/%08x/%04x (%08x/%04x)\r\n", m_machine.busy, m_machine.request, m_machine.event, (m_peripheral.central ? m_peripheral.central->m_machine.request : 0), (m_peripheral.central ? m_peripheral.central->m_machine.event : 0));
#endif

    if (!m_machine.busy) {
        
        mask = m_machine.request;
        event = m_machine.event;
        
        /* Cannot transmit */
        if (event & (BLE_EVENT_GATT_TX_POOL_AVAILABLE)) {
            mask &= ~(BLE_REQUEST_GATT_NOTIFY_VALUE);
        }

        /* Wait for completion event */
        if (event & (BLE_EVENT_GATT_SERVER_CONFIRMATION | BLE_EVENT_GATT_NOTIFICATION_COMPLETE)) {
            mask &= ~(BLE_REQUEST_GATT_NOTIFY_VALUE);
        }

        for (index = 0; index < BLE_NUM_LINK; index++) {
            device = m_machine.peer[((m_machine.index +1) + index) & (BLE_NUM_LINK-1)];

            if (device && device->m_machine.request) {
                device_mask = device->m_machine.request;
                device_event = device->m_machine.event;
                
                if (device_event & BLE_EVENT_GATT_PROC_COMPLETE) {
                    device_mask &= ~(BLE_REQUEST_GATT_DISCOVER_SERVICE_BY_UUID |
                                     BLE_REQUEST_GATT_DISCOVER_SERVICES |
                                     BLE_REQUEST_GATT_DISCOVER_CHARACTERISTICS |
                                     BLE_REQUEST_GATT_DISCOVER_DESCRIPTORS |
                                     BLE_REQUEST_GATT_READ_VALUE |
                                     BLE_REQUEST_GATT_READ_VALUE_LONG |
                                     BLE_REQUEST_GATT_WRITE_VALUE |
                                     BLE_REQUEST_GATT_WRITE_VALUE_LONG |
                                     BLE_REQUEST_GATT_WRITE_VALUE_WITHOUT_RESPONSE |
                                     BLE_REQUEST_GATT_WRITE_CONFIG);
                }

                if (device_mask & BLE_REQUEST_GAP_TERMINATE) {
                    device_mask = BLE_REQUEST_GAP_TERMINATE;
                }

                if (device_mask) {
                    mask |= device_mask;

                    m_machine.index = device->m_index;
                    m_machine.device = device;
                    break;
                }
            }
        }

        if (mask) {
            index = __builtin_ctz(mask);
            mask = (1ul << index);

            if (device && (device->m_machine.request & mask)) {
                armv7m_atomic_and(&device->m_machine.request, ~mask);
            } else {
                armv7m_atomic_and(&m_machine.request, ~mask);
            }
            
            switch (mask) {
            case BLE_REQUEST_HCI_LE_SET_PHY: {
                hci_le_set_phy_cp0 *cparam = (hci_le_set_phy_cp0*)m_machine.command.cparam;

                cparam->Connection_Handle = device->m_handle;
                cparam->ALL_PHYS = 0;
                cparam->TX_PHYS = m_connection.tx_phy;
                cparam->RX_PHYS = m_connection.rx_phy;
                cparam->PHY_options = m_connection.phy_options;
                
                m_machine.command.opcode = HCI_LE_SET_PHY;
                m_machine.command.clen = sizeof(hci_le_set_phy_cp0);
                break;
            }
 
            case BLE_REQUEST_HCI_LE_READ_PHY: {
                hci_le_read_phy_cp0 *cparam = (hci_le_read_phy_cp0*)m_machine.command.cparam;

                cparam->Connection_Handle = device->m_handle;
                
                m_machine.command.opcode = HCI_LE_READ_PHY;
                m_machine.command.clen = sizeof(hci_le_read_phy_cp0);
                break;
            }
                
            case BLE_REQUEST_L2CAP_CONNECTION_PARAMETER_UPDATE: {
                aci_l2cap_connection_parameter_update_req_cp0 *cparam = (aci_l2cap_connection_parameter_update_req_cp0*)m_machine.command.cparam;

                cparam->Connection_Handle = device->m_handle;
                cparam->Conn_Interval_Min = device->m_machine.params.interval_min;
                cparam->Conn_Interval_Max = device->m_machine.params.interval_max;
                cparam->Latency = device->m_machine.params.latency;
                cparam->Timeout_Multiplier = device->m_machine.params.timeout;
                
                m_machine.command.opcode = ACI_L2CAP_CONNECTION_PARAMETER_UPDATE_REQ;
                m_machine.command.clen = sizeof(aci_l2cap_connection_parameter_update_req_cp0);
                break;
            }

            case BLE_REQUEST_L2CAP_CONNECTION_PARAMETER_CONFIRM: {
                aci_l2cap_connection_parameter_update_resp_cp0 *cparam = (aci_l2cap_connection_parameter_update_resp_cp0*)m_machine.command.cparam;

                cparam->Connection_Handle = device->m_handle;
                cparam->Conn_Interval_Min = device->m_machine.params.interval_min;
                cparam->Conn_Interval_Max = device->m_machine.params.interval_max;
                cparam->Latency = device->m_machine.params.latency;
                cparam->Timeout_Multiplier = device->m_machine.params.timeout;
                cparam->Minimum_CE_Length = device->m_machine.params.min_ce_length;
                cparam->Maximum_CE_Length = device->m_machine.params.max_ce_length;
                cparam->Identifier = device->m_machine.params.sequence;
                cparam->Accept = device->m_machine.params.accept;
                
                m_machine.command.opcode = ACI_L2CAP_CONNECTION_PARAMETER_UPDATE_RESP;
                m_machine.command.clen = sizeof(aci_l2cap_connection_parameter_update_resp_cp0);
                break;
            }
                
            case BLE_REQUEST_GAP_CREATE_CONNECTION: {
                aci_gap_create_connection_cp0 *cparam = (aci_gap_create_connection_cp0*)m_machine.command.cparam;

                cparam->LE_Scan_Interval = m_central.scan_interval;
                cparam->LE_Scan_Window = m_central.scan_window;
                cparam->Peer_Address_Type = device->m_address.type();
                memcpy(&cparam->Peer_Address[0], device->m_address.data(), BLE_ADDRESS_SIZE);
                cparam->Own_Address_Type = ((m_options & BLE_OPTION_PRIVACY) ? 0x02 : m_address.type());
                cparam->Conn_Interval_Min = device->m_machine.params.interval_min;
                cparam->Conn_Interval_Max = device->m_machine.params.interval_max;
                cparam->Conn_Latency = device->m_machine.params.latency;
                cparam->Supervision_Timeout = device->m_machine.params.timeout;
                cparam->Minimum_CE_Length = device->m_machine.params.min_ce_length;
                cparam->Maximum_CE_Length = device->m_machine.params.max_ce_length;

                m_machine.command.opcode = ACI_GAP_CREATE_CONNECTION;
                m_machine.command.clen = sizeof(aci_gap_create_connection_cp0);
                break;
            }
              
            case BLE_REQUEST_GAP_TERMINATE: {
                aci_gap_terminate_cp0 *cparam = (aci_gap_terminate_cp0*)m_machine.command.cparam;

                cparam->Connection_Handle = device->m_handle;
                cparam->Reason = HCI_CONNECTION_TERMINATED_BY_LOCAL_HOST_ERR_CODE;
                
                m_machine.command.opcode = ACI_GAP_TERMINATE;
                m_machine.command.clen = sizeof(aci_gap_terminate_cp0);
                break;
            }

            case BLE_REQUEST_GAP_PASSKEY: {
                aci_gap_pass_key_resp_cp0 *cparam = (aci_gap_pass_key_resp_cp0*)m_machine.command.cparam;

                cparam->Connection_Handle = device->m_handle;
                cparam->Pass_Key = device->m_machine.passkey;
                
                m_machine.command.opcode = ACI_GAP_PASS_KEY_RESP;
                m_machine.command.clen = sizeof(aci_gap_pass_key_resp_cp0);
                break;
            }

            case BLE_REQUEST_GAP_PERIPHERAL_SECURITY: {
                aci_gap_peripheral_security_req_cp0 *cparam = (aci_gap_peripheral_security_req_cp0*)m_machine.command.cparam;

                cparam->Connection_Handle = device->m_handle;
                
                m_machine.command.opcode = ACI_GAP_PERIPHERAL_SECURITY_REQ;
                m_machine.command.clen = sizeof(aci_gap_peripheral_security_req_cp0);
                break;
            }

            case BLE_REQUEST_GAP_CENTRAL_SECURITY: {
                aci_gap_send_pairing_req_cp0 *cparam = (aci_gap_send_pairing_req_cp0*)m_machine.command.cparam;

                cparam->Connection_Handle = device->m_handle;
                cparam->Force_Rebond = 0;
                
                m_machine.command.opcode = ACI_GAP_SEND_PAIRING_REQ;
                m_machine.command.clen = sizeof(aci_gap_send_pairing_req_cp0);
                break;
            }
              
            case BLE_REQUEST_GAP_ALLOW_REBOND: {
                aci_gap_allow_rebond_cp0 *cparam = (aci_gap_allow_rebond_cp0*)m_machine.command.cparam;

                cparam->Connection_Handle = device->m_handle;
                
                m_machine.command.opcode = ACI_GAP_ALLOW_REBOND;
                m_machine.command.clen = sizeof(aci_gap_allow_rebond_cp0);
                break;
            }

            case BLE_REQUEST_GATT_NOTIFY_VALUE: {
                aci_gatt_update_char_value_ext_cp0 *cparam = (aci_gatt_update_char_value_ext_cp0*)m_machine.command.cparam;
                BLEServerCharacteristic *notify = m_machine.notify_current;

                m_machine.notify_count = m_machine.notify_length - m_machine.notify_offset;
                
                if (m_machine.notify_count > 243) {
                    m_machine.notify_count = 243;
                }
                
                cparam->Conn_Handle_To_Notify = 0x0000;
                cparam->Service_Handle = notify->m_parent->m_handle;
                cparam->Char_Handle = notify->m_handle;
                cparam->Update_Type = ((m_machine.notify_offset + m_machine.notify_count) == m_machine.notify_length) ? m_machine.notify_type: 0x00;
                cparam->Char_Length = m_machine.notify_length;
                cparam->Value_Offset = m_machine.notify_offset;
                cparam->Value_Length = m_machine.notify_count;
                memcpy(&cparam->Value[0], &notify->m_value_data[m_machine.notify_offset], m_machine.notify_count);
                
                m_machine.command.opcode = ACI_GATT_UPDATE_CHAR_VALUE_EXT;
                m_machine.command.clen = 12 + m_machine.notify_count;
                break;
            }
                    
            case BLE_REQUEST_GATT_SET_VALUE: {
                aci_gatt_update_char_value_ext_cp0 *cparam = (aci_gatt_update_char_value_ext_cp0*)m_machine.command.cparam;
                BLEServerCharacteristic *set = m_machine.set_current;

                m_machine.set_count = m_machine.set_length - m_machine.set_offset;
                
                if (m_machine.set_count > 243) {
                    m_machine.set_count = 243;
                }
                
                cparam->Conn_Handle_To_Notify = 0x0000;
                cparam->Service_Handle = set->m_parent->m_handle;
                cparam->Char_Handle = set->m_handle;
                cparam->Update_Type = 0x00;
                cparam->Char_Length = m_machine.set_length;
                cparam->Value_Offset = m_machine.set_offset;
                cparam->Value_Length = m_machine.set_count;
                memcpy(&cparam->Value[0], &m_machine.set_data[m_machine.set_offset], m_machine.set_count);
                
                m_machine.command.opcode = ACI_GATT_UPDATE_CHAR_VALUE_EXT;
                m_machine.command.clen = 12 + m_machine.set_count;
                break;
            }
                
            case BLE_REQUEST_GATT_ALLOW_READ: {
                aci_gatt_allow_read_cp0 *cparam = (aci_gatt_allow_read_cp0*)m_machine.command.cparam;

                cparam->Connection_Handle = device->m_handle;
                
                m_machine.command.opcode = ACI_GATT_ALLOW_READ;
                m_machine.command.clen = sizeof(aci_gatt_allow_read_cp0);
                break;
            }

            case BLE_REQUEST_GATT_MTU_EXCHANGE: {
                aci_gatt_exchange_config_cp0 *cparam = (aci_gatt_exchange_config_cp0*)m_machine.command.cparam;
                
                cparam->Connection_Handle = device->m_handle;

                m_machine.command.opcode = ACI_GATT_EXCHANGE_CONFIG;
                m_machine.command.clen = sizeof(aci_gatt_exchange_config_cp0);
                break;
            }
              
            case BLE_REQUEST_GATT_DISCOVER_SERVICE_BY_UUID: {
                aci_gatt_disc_primary_service_by_uuid_cp0 *cparam = (aci_gatt_disc_primary_service_by_uuid_cp0*)m_machine.command.cparam;
                
                device->m_machine.procedure = BLE_GATT_PROCEDURE_DISCOVER_SERVICE_BY_UUID;
                
                cparam->Connection_Handle = device->m_handle;

                if (device->m_machine.discovery_uuid.type() == BLE_UUID_TYPE_16) {
                    cparam->UUID_Type = UUID_TYPE_16;
                    memcpy(&cparam->UUID, device->m_machine.discovery_uuid.data(), 2);
                    
                    m_machine.command.opcode = ACI_GATT_DISC_PRIMARY_SERVICE_BY_UUID;
                    m_machine.command.clen = sizeof(aci_gatt_disc_primary_service_by_uuid_cp0) - sizeof(UUID_t) + 2;
                } else {
                    cparam->UUID_Type = UUID_TYPE_128;
                    memcpy(&cparam->UUID, device->m_machine.discovery_uuid.data(), 16);

                    m_machine.command.opcode = ACI_GATT_DISC_PRIMARY_SERVICE_BY_UUID;
                    m_machine.command.clen = sizeof(aci_gatt_disc_primary_service_by_uuid_cp0) - sizeof(UUID_t) + 16;
                }
                break;
            }

            case BLE_REQUEST_GATT_DISCOVER_SERVICES: {
                aci_gatt_disc_all_primary_services_cp0 *cparam = (aci_gatt_disc_all_primary_services_cp0*)m_machine.command.cparam;

                device->m_machine.procedure = BLE_GATT_PROCEDURE_DISCOVER_SERVICES;
                
                cparam->Connection_Handle = device->m_handle;
            
                m_machine.command.opcode = ACI_GATT_DISC_ALL_PRIMARY_SERVICES;
                m_machine.command.clen = sizeof(aci_gatt_disc_all_primary_services_cp0);
                break;
            }
                
            case BLE_REQUEST_GATT_DISCOVER_CHARACTERISTICS: {
                aci_gatt_disc_all_char_of_service_cp0 *cparam = (aci_gatt_disc_all_char_of_service_cp0*)m_machine.command.cparam;

                device->m_machine.procedure = BLE_GATT_PROCEDURE_DISCOVER_CHARACTERISTICS;
                
                cparam->Connection_Handle = device->m_handle;
                cparam->Start_Handle = device->m_machine.discovery_service->m_handle + 1;
                cparam->End_Handle = device->m_machine.discovery_service->m_end_group_handle;
            
                m_machine.command.opcode = ACI_GATT_DISC_ALL_CHAR_OF_SERVICE;
                m_machine.command.clen = sizeof(aci_gatt_disc_all_char_of_service_cp0);
                break;
            }
                
            case BLE_REQUEST_GATT_DISCOVER_DESCRIPTORS: {
                aci_gatt_disc_all_char_desc_cp0 *cparam = (aci_gatt_disc_all_char_desc_cp0*)m_machine.command.cparam;
                BLEClientService *service = device->m_machine.discovery_service;
                BLEClientCharacteristic *characteristic = device->m_machine.discovery_characteristic;

                device->m_machine.procedure = BLE_GATT_PROCEDURE_DISCOVER_DESCRIPTORS;
                
                cparam->Connection_Handle = device->m_handle;
                cparam->Char_Handle = characteristic->m_value_handle;
                cparam->End_Handle = (characteristic->m_sibling ? (characteristic->m_sibling->m_handle -1) : service->m_end_group_handle);
            
                m_machine.command.opcode = ACI_GATT_DISC_ALL_CHAR_DESC;
                m_machine.command.clen = sizeof(aci_gatt_disc_all_char_desc_cp0);
                break;
            }

            case BLE_REQUEST_GATT_CONFIRM_INDICATION: {
                aci_gatt_confirm_indication_cp0 *cparam = (aci_gatt_confirm_indication_cp0*)m_machine.command.cparam;
                
                cparam->Connection_Handle = device->m_handle;
            
                m_machine.command.opcode = ACI_GATT_CONFIRM_INDICATION;
                m_machine.command.clen = sizeof(aci_gatt_confirm_indication_cp0);
                break;
            }

            case BLE_REQUEST_GATT_READ_VALUE: {
                aci_gatt_read_char_value_cp0 *cparam = (aci_gatt_read_char_value_cp0*)m_machine.command.cparam;
                BLEClientCharacteristic *characteristic = device->m_machine.read_current;

                device->m_machine.procedure = BLE_GATT_PROCEDURE_READ_VALUE;
                
                cparam->Connection_Handle = device->m_handle;
                cparam->Attr_Handle = characteristic->m_value_handle;
            
                m_machine.command.opcode = ACI_GATT_READ_CHAR_VALUE;
                m_machine.command.clen = sizeof(aci_gatt_read_char_value_cp0);
                break;
            }

            case BLE_REQUEST_GATT_READ_VALUE_LONG: {
                aci_gatt_read_long_char_value_cp0 *cparam = (aci_gatt_read_long_char_value_cp0*)m_machine.command.cparam;
                BLEClientCharacteristic *characteristic = device->m_machine.read_current;

                device->m_machine.procedure = BLE_GATT_PROCEDURE_READ_VALUE_LONG;

                cparam->Connection_Handle = device->m_handle;
                cparam->Attr_Handle = characteristic->m_value_handle;
                cparam->Val_Offset = device->m_machine.read_offset;

                m_machine.command.opcode = ACI_GATT_READ_LONG_CHAR_VALUE;
                m_machine.command.clen = sizeof(aci_gatt_read_long_char_value_cp0);
                break;
            }

            case BLE_REQUEST_GATT_WRITE_VALUE: {
                aci_gatt_write_char_value_cp0 *cparam = (aci_gatt_write_char_value_cp0*)m_machine.command.cparam;
                BLEClientCharacteristic *characteristic = device->m_machine.write_current;

                device->m_machine.procedure = BLE_GATT_PROCEDURE_WRITE_VALUE;
                
                cparam->Connection_Handle = device->m_handle;
                cparam->Attr_Handle = characteristic->m_value_handle;
                cparam->Attribute_Val_Length = characteristic->m_value_length;
                memcpy(&cparam->Attribute_Val[0], &characteristic->m_value_data[0], characteristic->m_value_length);
            
                m_machine.command.opcode = ACI_GATT_WRITE_CHAR_VALUE;
                m_machine.command.clen = sizeof(aci_gatt_write_char_value_cp0) - sizeof(cparam->Attribute_Val) + cparam->Attribute_Val_Length;
                break;
            }
                
            case BLE_REQUEST_GATT_WRITE_VALUE_LONG: {
                aci_gatt_write_long_char_value_cp0 *cparam = (aci_gatt_write_long_char_value_cp0*)m_machine.command.cparam;
                BLEClientCharacteristic *characteristic = device->m_machine.write_current;

                device->m_machine.procedure = BLE_GATT_PROCEDURE_WRITE_VALUE_LONG;

                cparam->Connection_Handle = device->m_handle;
                cparam->Attr_Handle = characteristic->m_value_handle;
                cparam->Val_Offset = 0;
                cparam->Attribute_Val_Length = characteristic->m_value_length;
                memcpy(&cparam->Attribute_Val[0], &characteristic->m_value_data[0], characteristic->m_value_length);
            
                m_machine.command.opcode = ACI_GATT_WRITE_LONG_CHAR_VALUE;
                m_machine.command.clen = sizeof(aci_gatt_write_long_char_value_cp0) - sizeof(cparam->Attribute_Val) + cparam->Attribute_Val_Length;
                break;
            }

            case BLE_REQUEST_GATT_WRITE_VALUE_WITHOUT_RESPONSE: {
                aci_gatt_write_without_resp_cp0 *cparam = (aci_gatt_write_without_resp_cp0*)m_machine.command.cparam;
                BLEClientCharacteristic *characteristic = device->m_machine.write_current;

                device->m_machine.procedure = BLE_GATT_PROCEDURE_WRITE_VALUE_WITHOUT_RESPONSE;
                
                cparam->Connection_Handle = device->m_handle;
                cparam->Attr_Handle = characteristic->m_value_handle;
                cparam->Attribute_Val_Length = characteristic->m_value_length;
                memcpy(&cparam->Attribute_Val[0], &characteristic->m_value_data[0], characteristic->m_value_length);
            
                m_machine.command.opcode = ACI_GATT_WRITE_WITHOUT_RESP;
                m_machine.command.clen = sizeof(aci_gatt_write_without_resp_cp0) - sizeof(cparam->Attribute_Val) + cparam->Attribute_Val_Length;
                break;
            }

            case BLE_REQUEST_GATT_WRITE_CONFIG: {
                aci_gatt_write_char_desc_cp0 *cparam = (aci_gatt_write_char_desc_cp0*)m_machine.command.cparam;
                BLEClientCharacteristic *characteristic = device->m_machine.config_current;

                device->m_machine.procedure = BLE_GATT_PROCEDURE_WRITE_CONFIG;
                
                cparam->Connection_Handle = device->m_handle;
                cparam->Attr_Handle = characteristic->m_config_handle;
                cparam->Attribute_Val_Length = 1;
                cparam->Attribute_Val[0] = characteristic->m_config;
                
                m_machine.command.opcode = ACI_GATT_WRITE_CHAR_DESC; 
                m_machine.command.clen = sizeof(aci_gatt_write_char_desc_cp0) - sizeof(cparam->Attribute_Val) + cparam->Attribute_Val_Length;
                break;
            }
              
            default:
                break;
            }

            stm32wb_ipcc_ble_command(&m_machine.command);

            m_machine.busy = (index +1);
        }
    }

#if (BLE_TRACE_SUPPORTED_EXT == 1)
    armv7m_rtt_printf("PROCESS-ENTER %02x/%08x/%04x (%08x)\r\n", m_machine.busy, m_machine.request, m_machine.event, (m_peripheral.central ? m_peripheral.central->m_machine.request : 0));
#endif
}

/**********************************************************************************
 * BLE API
 */

BLECharacteristic::BLECharacteristic() {
    m_interface = &BLENullCharacteristic;
}

BLECharacteristic::BLECharacteristic(BLECharacteristicInterface *interface) {
    m_interface = interface;

    interface->reference();
}

BLECharacteristic::BLECharacteristic(const char *uuid, uint8_t properties, uint8_t permissions, const void *value, int valueLength, int valueSize, bool fixedLength) {
    m_interface = construct(BLEUuid(uuid), properties, permissions, value, valueLength, valueSize, fixedLength, false);
}

BLECharacteristic::BLECharacteristic(const BLEUuid &uuid, uint8_t properties, uint8_t permissions, const void *value, int valueLength, int valueSize, bool fixedLength) {
    m_interface = construct(uuid, properties, permissions, value, valueLength, valueSize, fixedLength, false);
}

BLECharacteristic::BLECharacteristic(const char *uuid, uint8_t properties, uint8_t permissions, int valueSize, bool fixedLength) {
    m_interface = construct(BLEUuid(uuid), properties, permissions, NULL, 0, valueSize, fixedLength, false);
}

BLECharacteristic::BLECharacteristic(const BLEUuid &uuid, uint8_t properties, uint8_t permissions, int valueSize, bool fixedLength) {
    m_interface = construct(uuid, properties, permissions, NULL, 0, valueSize, fixedLength, false);
}

BLECharacteristic::BLECharacteristic(const char* uuid, uint8_t properties, uint8_t permissions, const char* value) {
    int valueSize = strlen(value);

    m_interface = construct(BLEUuid(uuid), properties, permissions, (uint8_t*)value, valueSize, valueSize, true, true);
}

BLECharacteristic::BLECharacteristic(const BLEUuid &uuid, uint8_t properties, uint8_t permissions, const char* value) {
    int valueSize = strlen(value);

    m_interface = construct(uuid, properties, permissions, (uint8_t*)value, valueSize, valueSize, true, true);
}

BLECharacteristic::BLECharacteristic(const char* uuid, uint8_t properties, const char* value) {
    int valueSize = strlen(value);

    m_interface = construct(BLEUuid(uuid), properties, 0, (uint8_t*)value, valueSize, valueSize, true, true);
}

BLECharacteristic::BLECharacteristic(const BLEUuid &uuid, uint8_t properties, const char* value) {
    int valueSize = strlen(value);

    m_interface = construct(uuid, properties, 0, (uint8_t*)value, valueSize, valueSize, true, true);
}

BLECharacteristic::~BLECharacteristic() {
    (*m_interface).unreference();
}

BLECharacteristic::BLECharacteristic(const BLECharacteristic &other) {
    m_interface = other.m_interface;

    (*m_interface).reference();
}

BLECharacteristic &BLECharacteristic::operator=(const BLECharacteristic &other) {
    BLECharacteristicInterface *characteristic = m_interface;

    m_interface = other.m_interface;

    (*m_interface).reference();

    characteristic->unreference();

    return *this;
}

BLECharacteristic::BLECharacteristic(BLECharacteristic &&other) {
    m_interface = other.m_interface;

    other.m_interface = &BLENullCharacteristic;
}

BLECharacteristic &BLECharacteristic::operator=(BLECharacteristic &&other) {
    BLECharacteristicInterface *characteristic = m_interface;

    m_interface = other.m_interface;

    other.m_interface = &BLENullCharacteristic;

    characteristic->unreference();

    return *this;
}

BLECharacteristic::operator bool() const {
    return (*m_interface);
}

const BLEUuid BLECharacteristic::uuid() const {
    return (*m_interface).uuid();
}

uint8_t BLECharacteristic::properties() const {
    return (*m_interface).properties();
}

bool BLECharacteristic::fixedLength() const {
    return (*m_interface).fixedLength();
}

int BLECharacteristic::valueSize() {
    return (*m_interface).valueSize();
}

int BLECharacteristic::getValue(void *value, int size) {
    return (*m_interface).getValue(value, size);
}

bool BLECharacteristic::setValue(const void *value, int length) {
    return (*m_interface).setValue(value, length);
}

bool BLECharacteristic::notifyValue(const void *value, int length) {
    return (*m_interface).notifyValue(value, length, Callback());
}

bool BLECharacteristic::notifyValue(const void *value, int length, void(*callback)(void)) {
    return (*m_interface).notifyValue(value, length, Callback(callback));
}

bool BLECharacteristic::notifyValue(const void *value, int length, Callback callback) {
    return (*m_interface).notifyValue(value, length, callback);
}

int BLECharacteristic::readValue(void *value, int size) {
    k_task_t *task_self = k_task_self();
    
    if (!task_self) {
        return 0;
    }
  
    if (!(*m_interface).read(Callback(done_callback, (void*)task_self))) {
        return 0;
    }
    
    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

    return (*m_interface).getValue(value, size);
}

bool BLECharacteristic::writeValue(const void *value, int length, BLEWriteType writeType) {
    return (*m_interface).writeValue(value, length, writeType, Callback());
}

bool BLECharacteristic::writeValue(const void *value, int length, BLEWriteType writeType, void(*callback)(void)) {
    return (*m_interface).writeValue(value, length, writeType, Callback(callback));
}

bool BLECharacteristic::writeValue(const void *value, int length, BLEWriteType writeType, Callback callback) {
    return (*m_interface).writeValue(value, length, writeType, callback);
}

bool BLECharacteristic::read() {
    return (*m_interface).read(Callback());
}

bool BLECharacteristic::read(void(*callback)(void)) {
    return (*m_interface).read(Callback(callback));
}

bool BLECharacteristic::read(Callback callback) {
    return (*m_interface).read(callback);
}

bool BLECharacteristic::subscribe() {
    return (*m_interface).subscribe();
}

bool BLECharacteristic::unsubscribe() {
    return (*m_interface).unsubscribe();
}

uint8_t BLECharacteristic::status() {
    return (*m_interface).status();
}

bool BLECharacteristic::busy() {
    return (*m_interface).busy();
}

bool BLECharacteristic::written() {
    return (*m_interface).written();
}

bool BLECharacteristic::subscribed() {
    return (*m_interface).subscribed();
}

void BLECharacteristic::onNotify(void(*callback)(void)) {
    (*m_interface).onNotify(Callback(callback));
}

void BLECharacteristic::onNotify(Callback callback) {
    (*m_interface).onNotify(callback);
}

void BLECharacteristic::onRead(void(*callback)(void)) {
    (*m_interface).onRead(Callback(callback));
}

void BLECharacteristic::onRead(Callback callback) {
    (*m_interface).onRead(callback);
}

void BLECharacteristic::onWrite(void(*callback)(void)) {
    (*m_interface).onWrite(Callback(callback));
}

void BLECharacteristic::onWrite(Callback callback) {
    (*m_interface).onWrite(callback);
}

void BLECharacteristic::onSubscribe(void(*callback)(void)) {
    (*m_interface).onSubscribe(Callback(callback));
}

void BLECharacteristic::onSubscribe(Callback callback) {
    (*m_interface).onSubscribe(callback);
}

BLECharacteristicInterface *BLECharacteristic::construct(const BLEUuid &uuid, uint8_t properties, uint8_t permissions, const void *value, int valueLength, int valueSize, bool fixedLength, bool constant) {
    BLEServerCharacteristic *characteristic;
    uint8_t *data;

    if (valueSize > BLE_MAX_ATT_SIZE) {
        return &BLENullCharacteristic;
    }

    if ((properties & BLE_PROPERTY_NOTIFY) && (properties & BLE_PROPERTY_INDICATE)) {
        return &BLENullCharacteristic;
    }
    
#if 0    
    if ((properties & (BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) && (valueSize > BLE_MIN_ATT_SIZE)) {
        return &BLENullCharacteristic;
    }
#endif
    
    if (!uuid) {
        return &BLENullCharacteristic;
    }

    if (constant) {
        if (properties & (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE | BLE_PROPERTY_WRITE | BLE_PROPERTY_NOTIFY | BLE_PROPERTY_INDICATE)) {
            return &BLENullCharacteristic;
        }

        data = (uint8_t*)allocate_noconst(value, valueSize);
    } else {
        data = (uint8_t*)malloc(((properties & (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE | BLE_PROPERTY_WRITE)) ? (2 * valueSize) : valueSize));

        if (value) {
            memcpy(data, value, valueLength);
        }
    }

    if (data) {
        characteristic = new BLEServerCharacteristic(uuid, properties, permissions, data, valueLength, valueSize, fixedLength, constant);

        if (characteristic) {
            return characteristic;
        }
    }

    deallocate_noconst(data);
    
    return &BLENullCharacteristic;
}

BLEService::BLEService() {
    m_interface = &BLENullService;
}

BLEService::BLEService(BLEServiceInterface *interface) {
    m_interface = interface;

    interface->reference();
}

BLEService::BLEService(const char *uuid) {
    m_interface = construct(BLEUuid(uuid));
}

BLEService::BLEService(const BLEUuid &uuid) {
    m_interface = construct(uuid);
}

BLEService::~BLEService() {
    (*m_interface).unreference();
}

BLEService::BLEService(const BLEService &other) {
    m_interface = other.m_interface;

    (*m_interface).reference();
}

BLEService &BLEService::operator=(const BLEService &other) {
    BLEServiceInterface *service = m_interface;

    m_interface = other.m_interface;

    (*m_interface).reference();

    service->unreference();

    return *this;
}

BLEService::BLEService(BLEService &&other) {
    m_interface = other.m_interface;

    other.m_interface = &BLENullService;
}

BLEService &BLEService::operator=(BLEService &&other) {
    BLEServiceInterface *service = m_interface;

    m_interface = other.m_interface;

    other.m_interface = &BLENullService;

    service->unreference();

    return *this;
}

BLEService::operator bool() const {
    return (*m_interface);
}

const BLEUuid BLEService::uuid() const {
    return (*m_interface).uuid();
}

unsigned int BLEService::characteristicCount() {
    return (*m_interface).characteristicCount();
}

BLECharacteristic BLEService::characteristic(unsigned int index) {
    return (*m_interface).characteristic(index);
}

BLECharacteristic BLEService::characteristic(const char *uuid) {
    return (*m_interface).characteristic(BLEUuid(uuid));
}

BLECharacteristic BLEService::characteristic(const BLEUuid &uuid) {
    return (*m_interface).characteristic(uuid);
}

bool BLEService::addCharacteristic(BLECharacteristic &characteristic) {
    return (*m_interface).addCharacteristic(characteristic);
}

BLEServiceInterface *BLEService::construct(const BLEUuid &uuid) {
    BLEServerService *service;
    
    if (!uuid) {
        return &BLENullService;
    }
    
    service = new BLEServerService(uuid);

    if (service) {
        return service;
    }
    
    return &BLENullService;
}

BLEDevice::BLEDevice() {
    m_interface = &BLENullDevice;
}

BLEDevice::BLEDevice(BLEDeviceInterface *interface) {
    m_interface = interface;

    interface->reference();
}

BLEDevice::~BLEDevice() {
    m_interface->unreference();
}

BLEDevice::BLEDevice(const BLEDevice &other) {
    m_interface = other.m_interface;

    (*m_interface).reference();
}

BLEDevice &BLEDevice::operator=(const BLEDevice &other) {
    BLEDeviceInterface *device = m_interface;

    m_interface = other.m_interface;

    (*m_interface).reference();

    device->unreference();
    
    return *this;
}

BLEDevice::BLEDevice(BLEDevice &&other) {
    m_interface = other.m_interface;

    other.m_interface = &BLENullDevice;
}

BLEDevice &BLEDevice::operator=(BLEDevice &&other) {
    BLEDeviceInterface *device = m_interface;

    m_interface = other.m_interface;

    other.m_interface = &BLENullDevice;

    device->unreference();

    return *this;
}

BLEDevice::operator bool() const {
    return (*m_interface);
}

const BLEAddress BLEDevice::address() const {
    return (*m_interface).address();
}

int BLEDevice::mtu() {
    return (*m_interface).mtu();
}

void BLEDevice::disconnect() {
    (*m_interface).disconnect();
}

bool BLEDevice::connected() {
    return (*m_interface).connected();
}

bool BLEDevice::discover() {
    k_task_t *task_self = k_task_self();

    if (!task_self) {
        return false;
    }
    
    if (!((*m_interface).discover(Callback(done_callback, (void*)task_self)))) {
        return false;
    }

    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

    return true;
}

bool BLEDevice::discover(void(*callback)(void)) {
    return (*m_interface).discover(Callback(callback));
}

bool BLEDevice::discover(Callback callback) {
    return (*m_interface).discover(callback);
}

bool BLEDevice::discoverServices() {
    k_task_t *task_self = k_task_self();

    if (!task_self) {
        return false;
    }
    
    if (!((*m_interface).discoverServices(Callback(done_callback, (void*)task_self)))) {
        return false;
    }

    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

    return true;
}

bool BLEDevice::discoverServices(void(*callback)(void)) {
    return (*m_interface).discoverServices(Callback(callback));
}

bool BLEDevice::discoverServices(Callback callback) {
  return (*m_interface).discoverServices(callback);
}

bool BLEDevice::discoverCharacteristics(const char *uuid) {
    k_task_t *task_self = k_task_self();

    if (!task_self) {
        return false;
    }
    
    if (!((*m_interface).discoverCharacteristics(BLEUuid(uuid), Callback(done_callback, (void*)task_self)))) {
        return false;
    }

    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

    return true;
}

bool BLEDevice::discoverCharacteristics(const char *uuid, void(*callback)(void)) {
    return (*m_interface).discoverCharacteristics(BLEUuid(uuid), Callback(callback));
}

bool BLEDevice::discoverCharacteristics(const char *uuid, Callback callback) {
    return (*m_interface).discoverCharacteristics(BLEUuid(uuid), callback);
}

bool BLEDevice::discoverCharacteristics(const BLEUuid &uuid) {
    k_task_t *task_self = k_task_self();

    if (!task_self) {
        return false;
    }
    
    if (!((*m_interface).discoverCharacteristics(uuid, Callback(done_callback, (void*)task_self)))) {
        return false;
    }

    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

    return true;
}

bool BLEDevice::discoverCharacteristics(const BLEUuid &uuid, void(*callback)(void)) {
    return (*m_interface).discoverCharacteristics(uuid, Callback(callback));
}

bool BLEDevice::discoverCharacteristics(const BLEUuid &uuid, Callback callback) {
    return (*m_interface).discoverCharacteristics(uuid, callback);
}

bool BLEDevice::discoverCharacteristics(BLEService service) {
    k_task_t *task_self = k_task_self();

    if (!task_self) {
        return false;
    }
    
    if (!((*m_interface).discoverCharacteristics(service, Callback(done_callback, (void*)task_self)))) {
        return false;
    }

    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

    return true;
}

bool BLEDevice::discoverCharacteristics(BLEService service, void(*callback)(void)) {
    return (*m_interface).discoverCharacteristics(service, Callback(callback));
}

bool BLEDevice::discoverCharacteristics(BLEService service, Callback callback) {
    return (*m_interface).discoverCharacteristics(service, callback);
}

unsigned int BLEDevice::serviceCount() {
    return (*m_interface).serviceCount();
}

BLEService BLEDevice::service(unsigned int index) {
    return (*m_interface).service(index);
}  

BLEService BLEDevice::service(const char *uuid) {
    return (*m_interface).service(BLEUuid(uuid));
}

BLEService BLEDevice::service(const BLEUuid &uuid) {
    return (*m_interface).service(uuid);
}

void BLEDevice::getPhys(BLEPhy &tx_phy, BLEPhy &rx_phy) {
    (*m_interface).getPhys(tx_phy, rx_phy);
}

bool BLEDevice::setConnectionInterval(uint16_t interval_min, uint16_t interval_max) {
    uint16_t latency, timeout;

    latency = 0;
    timeout = ((((1 + latency) * (2 * interval_max * 125 + 124)) / 100) + 9) / 10;

    if (timeout < 0x00c8) {
        timeout = 0x00c8;
    }
    
    return setConnectionParameters(interval_min, interval_max, latency, timeout);
}

bool BLEDevice::setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency) {
    uint16_t timeout;

    timeout = ((((1 + latency) * (2 * interval_max * 125 + 124)) / 100) + 9) / 10;

    if (timeout < 0x00c8) {
        timeout = 0x00c8;
    }

    return setConnectionParameters(interval_min, interval_max, latency, timeout);
}

bool BLEDevice::setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout) {
    return (*m_interface).setConnectionParameters(interval_min, interval_max, latency, timeout);
}

void BLEDevice::getConnectionParameters(uint16_t &connectionInterval, uint16_t &latency, uint16_t &timeout) {
    (*m_interface).getConnectionParameters(connectionInterval, latency, timeout);
}

void BLEDevice::onMTUExchange(void(*callback)(void)) {
    (*m_interface).onMTUExchange(Callback(callback));
}

void BLEDevice::onMTUExchange(Callback callback) {
    (*m_interface).onMTUExchange(callback);
}

void BLEDevice::onConnectionParameters(void(*callback)(void)) {
    (*m_interface).onConnectionParameters(Callback(callback));
}

void BLEDevice::onConnectionParameters(Callback callback) {
    (*m_interface).onConnectionParameters(callback);
}

void BLEDevice::onDisconnect(void(*callback)(void)) {
    (*m_interface).onDisconnect(Callback(callback));
}

void BLEDevice::onDisconnect(Callback callback) {
    (*m_interface).onDisconnect(callback);
}

bool BLEClass::begin() {
    return BLEHost.begin((BLE_OPTION_ROLE_PERIPHERAL | BLE_OPTION_ROLE_CENTRAL), BLE_MAX_ATT_MTU);
}

bool BLEClass::begin(uint32_t options) {
    return BLEHost.begin(options, BLE_MAX_ATT_MTU);
}

bool BLEClass::begin(uint32_t options, int mtu) {
    return BLEHost.begin(options, mtu);
}

void BLEClass::end() {
    BLEHost.end();
}

const BLEAddress BLEClass::address() {
    return BLEHost.address();
}

bool BLEClass::setTxPowerLevel(int txPower) {
    return BLEHost.setTxPowerLevel(txPower);
}

bool BLEClass::setPreferredPhy(BLEPhy txPhy, BLEPhy rxPhy, BLEPhyOption phyOptions) {
    return BLEHost.setPreferredPhy(txPhy, rxPhy, phyOptions);
}

bool BLEClass::setConnectionInterval(uint16_t interval_min, uint16_t interval_max) {
    uint16_t latency, timeout;

    latency = 0;
    timeout = ((((1 + latency) * (2 * interval_max * 125 + 124)) / 100) + 9) / 10;

    if (timeout < 0x00c8) {
        timeout = 0x00c8;
    }
    
    return setConnectionParameters(interval_min, interval_max, latency, timeout);
}

bool BLEClass::setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency) {
    uint16_t timeout;

    timeout = ((((1 + latency) * (2 * interval_max * 125 + 124)) / 100) + 9) / 10;

    if (timeout < 0x00c8) {
        timeout = 0x00c8;
    }

    return setConnectionParameters(interval_min, interval_max, latency, timeout);
}

bool BLEClass::setConnectionParameters(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout) {
    return BLEHost.setConnectionParameters(interval_min, interval_max, latency, timeout);
}

bool BLEClass::setDeviceName(const char *deviceName) {
    return BLEHost.setDeviceName(deviceName);
}

bool BLEClass::setAppearance(BLEAppearance appearance) {
    return BLEHost.setAppearance(appearance);
}

bool BLEClass::setSecurity(BLESecurity security) {
    return BLEHost.setSecurity(security);
}

bool BLEClass::setPasskey(uint32_t passkey) {
    return BLEHost.setPasskey(passkey);
}

bool BLEClass::setBonding(bool bonding) {
    return BLEHost.setBonding(bonding);
}

void BLEClass::setConnectable(bool connectable) {
    BLEHost.setConnectable(connectable);
}

void BLEClass::setDiscoverable(BLEDiscoverable discoverable) {
    BLEHost.setDiscoverable(discoverable);
}

bool BLEClass::setAdvertisingInterval(uint16_t advertisingInterval) {
    return BLE.setAdvertisingInterval(advertisingInterval, advertisingInterval);
}

bool BLEClass::setAdvertisingInterval(uint16_t minimumAdvertisingInterval, uint16_t maximumAdvertisingInterval) {
    return BLEHost.setAdvertisingInterval(minimumAdvertisingInterval, maximumAdvertisingInterval);
}

bool BLEClass::setAdvertisingData(const BLEAdvertisingData &advertising_data) {
    return BLEHost.setAdvertisingData(advertising_data);
}

void BLEClass::setAdvertisingData() {
    BLEHost.setAdvertisingData();
}

bool BLEClass::setScanResponseData(const BLEAdvertisingData &scan_response_data) {
    return BLEHost.setScanResponseData(scan_response_data);
}

void BLEClass::setScanResponseData() {
    BLEHost.setScanResponseData();
}

bool BLEClass::setLocalName(const char *localName) {
    return BLEHost.setLocalName(localName);
}

bool BLEClass::setServiceUuid(const char *uuid) {
    return BLEHost.setServiceUuid(BLEUuid(uuid));
}

bool BLEClass::setServiceUuid(const BLEUuid &uuid) {
    return BLEHost.setServiceUuid(uuid);
}

bool BLEClass::setServiceSolicitation(const char *uuid) {
    return BLEHost.setServiceSolicitation(BLEUuid(uuid));
}

bool BLEClass::setServiceSolicitation(const BLEUuid &uuid) {
    return BLEHost.setServiceSolicitation(uuid);
}

bool BLEClass::setServiceData(const char *uuid, const uint8_t data[], int length) {
    return BLEHost.setServiceData(BLEUuid(uuid), data, length);
}

bool BLEClass::setServiceData(const BLEUuid &uuid, const uint8_t data[], int length) {
    return BLEHost.setServiceData(uuid, data, length);
}

bool BLEClass::setManufacturerData(const uint8_t data[], int length) {
    return BLEHost.setManufacturerData(data, length);
}

bool BLEClass::setBeacon(const char *uuid, uint16_t major, uint16_t minor, int rssi) {
    return BLEHost.setBeacon(BLEUuid(uuid), major, minor, rssi);
}

bool BLEClass::setBeacon(const BLEUuid &uuid, uint16_t major, uint16_t minor, int rssi) {
    return BLEHost.setBeacon(uuid, major, minor, rssi);
}

bool BLEClass::advertise() {
    return BLEHost.advertise();
}

void BLEClass::stopAdvertise() {
    BLEHost.stopAdvertise();
}

bool BLEClass::advertising() {
    return BLEHost.advertising();
}

BLEDevice BLEClass::central() {
    return BLEHost.central();
}

bool BLEClass::connected() {
    return BLEHost.connected();
}

void BLEClass::disconnect() {
    BLEHost.disconnect();
}

bool BLEClass::setScanInterval(uint16_t scanInterval) {
    return BLEHost.setScanInterval(scanInterval);
}

bool BLEClass::setScanWindow(uint16_t scanWindow) {
    return BLEHost.setScanWindow(scanWindow);
}

void BLEClass::setScanType(BLEScanType scanType) {
    BLEHost.setScanType(scanType);
}

bool BLEClass::scan(uint8_t scanMode) {
    return BLEHost.scan(scanMode);
}

void BLEClass::stopScan() {
    BLEHost.stopScan();
}

bool BLEClass::scanning() {
    return BLEHost.scanning();
}

unsigned int BLEClass::reportCount() {
    return BLEHost.reportCount();
}

bool BLEClass::report(BLEAddress &address, int &rssi, BLEScanEvent &event, BLEAdvertisingData &data) {
    return BLEHost.report(address, rssi, event, data);
}

BLEDevice BLEClass::connect(BLEAddress address, uint32_t passkey) {
    k_task_t *task_self = k_task_self();
    
    if (!task_self) {
        return BLEDevice();
    }
    
    BLEDevice device = BLEHost.connect(address, passkey, Callback(done_callback, (void*)task_self));
    
    k_event_receive(WIRING_EVENT_TRANSIENT, (K_EVENT_ANY | K_EVENT_CLEAR), K_TIMEOUT_FOREVER, NULL);

    return device;
}

BLEDevice BLEClass::connect(BLEAddress address, uint32_t passkey, void(*callback)(void)) {
    return BLEHost.connect(address, passkey, Callback(callback));
}

BLEDevice BLEClass::connect(BLEAddress address, uint32_t passkey, Callback callback) {
    return BLEHost.connect(address, passkey, callback);
}

bool BLEClass::activate() {
    return BLEHost.activate();
}

unsigned int BLEClass::serviceCount() {
    return BLEHost.serviceCount();
}

BLEService BLEClass::service(unsigned int index) {
    return BLEHost.service(index);
}

BLEService BLEClass::service(const char *uuid) {
    return BLEHost.service(BLEUuid(uuid));
}

BLEService BLEClass::service(const BLEUuid &uuid) {
    return BLEHost.service(uuid);
}

bool BLEClass::addService(BLEService &service) {
    return BLEHost.addService(service);
}

int BLEClass::queryBondStorage(BLEAddress address[], int count) {
    return BLEHost.queryBondStorage(address, count);
}

bool BLEClass::removeBondStorage(const BLEAddress &address) {
    return BLEHost.removeBondStorage(address);
}

bool BLEClass::clearBondStorage() {
    return BLEHost.clearBondStorage();
}

bool BLEClass::testTransmit(int channel, int length, BLEPayload payload, BLEPhy phy) {
    return BLEHost.testTransmit(channel, length, payload, phy);
}

bool BLEClass::testReceive(int channel, BLEPhy phy, int modulation) {
    return BLEHost.testReceive(channel, phy, modulation);
}

uint32_t BLEClass::testStop() {
    return BLEHost.testStop();
}

bool BLEClass::testing() {
    return BLEHost.testing();
}

void BLEClass::onStop(void(*callback)(void)) {
    BLEHost.onStop(Callback(callback));
}

void BLEClass::onStop(Callback callback) {
    BLEHost.onStop(callback);
}

void BLEClass::onReport(void(*callback)(void)) {
    BLEHost.onReport(Callback(callback));
}

void BLEClass::onReport(Callback callback) {
    BLEHost.onReport(callback);
}

void BLEClass::onConnect(void(*callback)(void)) {
    BLEHost.onConnect(Callback(callback));
}

void BLEClass::onConnect(Callback callback) {
    BLEHost.onConnect(callback);
}

void BLEClass::onDisconnect(void(*callback)(void)) {
    BLEHost.onDisconnect(Callback(callback));
}

void BLEClass::onDisconnect(Callback callback) {
    BLEHost.onDisconnect(callback);
}

BLEClass BLE;




static const char *BLEUartServiceUUIDs[] = {
    "00000000-000E-11E1-9AB4-0002A5D5C51B",
    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E",
    "0000FFE0-0000-1000-8000-00805F9B34FB",
};

BLEUart::BLEUart(BLEUartProtocol protocol) :
    BLEService(BLEUartServiceUUIDs[(unsigned int)protocol]),
    m_protocol(protocol)
{
    switch (protocol) {
    case BLE_UART_PROTOCOL_BLUEST:
        m_packet_size = 20;

        m_rx_characteristic = BLECharacteristic("00000001-000E-11E1-AC36-0002A5D5C51B", (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE | BLE_PROPERTY_NOTIFY), 0, m_packet_size, false);
        m_tx_characteristic = m_rx_characteristic;
        m_tx2_characteristic = BLECharacteristic("00000002-000E-11E1-AC36-0002A5D5C51B", BLE_PROPERTY_NOTIFY, 0, m_packet_size, false);

        addCharacteristic(m_rx_characteristic);
        addCharacteristic(m_tx2_characteristic);
        break;
        
    case BLE_UART_PROTOCOL_NORDIC:
        m_packet_size = 20;

        m_rx_characteristic = BLECharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLE_PROPERTY_WRITE_WITHOUT_RESPONSE, 0, m_packet_size, false);
        m_tx_characteristic = BLECharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLE_PROPERTY_NOTIFY, 0, m_packet_size, false);

        addCharacteristic(m_rx_characteristic);
        addCharacteristic(m_tx_characteristic);
        break;

    case BLE_UART_PROTOCOL_HMSOFT:
        m_packet_size = 20;

        m_rx_characteristic = BLECharacteristic("0000FFE1-0000-1000-8000-00805F9B34FB", (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE | BLE_PROPERTY_NOTIFY), 0, m_packet_size, false);
        m_tx_characteristic = m_rx_characteristic;

        addCharacteristic(m_rx_characteristic);
        break;
    }
    
    m_rx_characteristic.onWrite(Callback(&BLEUart::receive, this));
    m_tx_characteristic.onSubscribe(Callback(&BLEUart::connect, this));

    m_receive_callback = Callback();
}

BLEUart::~BLEUart() {
}

BLEUart::operator bool() {
    return m_connected;
}

int BLEUart::available() {
    return m_rx_count;
}

int BLEUart::peek() {
    if (m_rx_count == 0) {
        return -1;
    }
        
    return m_rx_data[m_rx_read];
}

int BLEUart::read() {
    uint32_t rx_read, rx_read_next;
    uint8_t data;

    do {
        if (!m_rx_count) {
            return -1;
        }
        
        rx_read = m_rx_read;
        rx_read_next = (unsigned int)(rx_read + 1) & (BLE_UART_RX_BUFFER_SIZE -1);
    } while (armv7m_atomic_cash(&m_rx_read, rx_read, rx_read_next) != rx_read);

    data = m_rx_data[rx_read];
    
    armv7m_atomic_subh(&m_rx_count, 1);

    return data;
}

int BLEUart::read(uint8_t *buffer, size_t size) {
    uint32_t count, rx_count, rx_read, rx_read_next;

    count = 0;

    while (count < size) {
        do {
            rx_count = m_rx_count;

            if (!rx_count) {
                return count;
            }
            
            rx_read = m_rx_read;
            
            if (rx_count > (BLE_UART_RX_BUFFER_SIZE - rx_read)) {
                rx_count = (BLE_UART_RX_BUFFER_SIZE - rx_read);
            }
            
            if (rx_count > (size - count)) {
                rx_count = (size - count);
            }

            rx_read_next = (unsigned int)(rx_read + count) & (BLE_UART_RX_BUFFER_SIZE -1);
        } while (armv7m_atomic_cash(&m_rx_read, rx_read, rx_read_next) != rx_read);
    
        memcpy(&buffer[count], &m_rx_data[rx_read], rx_count);
        count += rx_count;
    
        armv7m_atomic_subh(&m_rx_count, rx_count);
    }

    return count;
}

int BLEUart::availableForWrite() {
    return BLE_UART_TX_BUFFER_SIZE - m_tx_count;
}

size_t BLEUart::write(uint8_t data) {
    return write(&data, 1);
}

size_t BLEUart::write(const uint8_t *buffer, size_t size) {
    uint32_t count, tx_read, tx_write, tx_write_next, tx_count, tx_size;

    if (size == 0) {
        return 0;
    }

    if (!m_connected) {
        return size;
    }
    
    count = 0;

    while (count < size) {
        tx_count = BLE_UART_TX_BUFFER_SIZE - m_tx_count;

        if (!tx_count) {
            if (m_nonblocking || armv7m_core_is_in_interrupt()) {
                break;
            }

            if (!armv7m_atomic_casb(&m_tx_busy, false, true) == false) {
                tx_size = m_tx_count;
                tx_read = m_tx_read;

                if (tx_size > (BLE_UART_TX_BUFFER_SIZE - tx_read)) {
                    tx_size = (BLE_UART_TX_BUFFER_SIZE - tx_read);
                }
                
                if (tx_size > m_packet_size) {
                    tx_size = m_packet_size;
                }
                
                m_tx_size = tx_size;
                
                while (!m_tx_characteristic.notifyValue(&m_tx_data[tx_read], tx_size, Callback(&BLEUart::transmit, this))) { }
            }

            while (m_tx_busy) { __WFE(); }
        } else {
            tx_write = m_tx_write;
            
            if (tx_count > (BLE_UART_TX_BUFFER_SIZE - tx_write)) {
                tx_count = (BLE_UART_TX_BUFFER_SIZE - tx_write);
            }
            
            if (tx_count > (size - count)) {
                tx_count = (size - count);
            }
            
            tx_write_next = (unsigned int)(tx_write + tx_count) & (BLE_UART_TX_BUFFER_SIZE -1);
            
            if (armv7m_atomic_cash(&m_tx_write, tx_write, tx_write_next) == tx_write) {
                memcpy(&m_tx_data[tx_write], &buffer[count], tx_count);
                count += tx_count;

                armv7m_atomic_addh(&m_tx_count, tx_count);
            }
        }
    }

    if (armv7m_atomic_casb(&m_tx_busy, false, true) == false) {
        tx_size = m_tx_count;
        
        if (tx_size) {
            tx_read = m_tx_read;
            
            if (tx_size > (BLE_UART_TX_BUFFER_SIZE - tx_read)) {
                tx_size = (BLE_UART_TX_BUFFER_SIZE - tx_read);
            }
            
            if (tx_size > m_packet_size) {
                tx_size = m_packet_size;
            }
            
            m_tx_size = tx_size;
            
            if (!m_tx_characteristic.notifyValue(&m_tx_data[tx_read], tx_size, Callback(&BLEUart::transmit, this))) {
                m_tx_busy = false;
            }
        } else {
            m_tx_busy = false;
        }
    }

    return count;
}

void BLEUart::flush() {
    if (!armv7m_core_is_in_interrupt()) {
        while (m_tx_busy) {
            __WFE();
        }
    }
}

void BLEUart::transmit() {
    unsigned int tx_read, tx_size;

    m_tx_read = (m_tx_read + m_tx_size) & (BLE_UART_TX_BUFFER_SIZE -1);
    armv7m_atomic_subh(&m_tx_count, m_tx_size);
        
    m_tx_size = 0;

    tx_size = m_tx_count;

    if (tx_size) {
        tx_read = m_tx_read;

        if (tx_size > (BLE_UART_TX_BUFFER_SIZE - tx_read)) {
            tx_size = (BLE_UART_TX_BUFFER_SIZE - tx_read);
        }
        
        if (tx_size > m_packet_size) {
            tx_size = m_packet_size;
        }
        
        m_tx_size = tx_size;
        
        while (!m_tx_characteristic.notifyValue(&m_tx_data[tx_read], tx_size, Callback(&BLEUart::transmit, this))) {
            receive();
        }
    } else {
        m_tx_busy = false;
    }
}

void BLEUart::receive() {
    uint32_t rx_count, rx_size, rx_write;
    uint8_t rx_data[m_packet_size];

    if (m_rx_characteristic.written()) {
        rx_count = m_rx_characteristic.getValue(rx_data, sizeof(rx_data));

        rx_write = m_rx_write;
        
        if (rx_count) {
            if (rx_count > (unsigned)(BLE_UART_RX_BUFFER_SIZE - m_rx_count)) {
                rx_count = (BLE_UART_RX_BUFFER_SIZE - m_rx_count);
            }
            
            rx_size = rx_count;
            
            if (rx_size > (BLE_UART_RX_BUFFER_SIZE - rx_write)) {
                rx_size = BLE_UART_RX_BUFFER_SIZE - rx_write;
            }
            
            if (rx_size) {
                memcpy(&m_rx_data[rx_write], &rx_data[0], rx_size);
                
                rx_count -= rx_size;
                rx_write += rx_size;
                
                if (rx_write == BLE_UART_RX_BUFFER_SIZE) {
                    rx_write = 0;
                }
            }
        
            if (rx_count) {
                memcpy(&m_rx_data[rx_write], &rx_data[rx_size], rx_count);
            
                rx_write += rx_count;
                
                if (rx_write == BLE_UART_RX_BUFFER_SIZE) {
                    rx_write = 0;
                }
            }

            armv7m_atomic_addh(&m_rx_count, (rx_size + rx_count));

            m_rx_write = rx_write;
            
            m_receive_callback();
        }
    }
}

void BLEUart::connect() {
    m_connected = !!m_tx_characteristic.subscribed();
}

void BLEUart::setNonBlocking(bool enabled) {
    m_nonblocking = enabled;
}

void BLEUart::onReceive(void(*callback)(void)) {
    onReceive(Callback(callback));
}

void BLEUart::onReceive(Callback callback) {
    m_receive_callback = callback;
}

#define BLE_OTA_PROTOCOL_WB                        0
#define BLE_OTA_PROTOCOL_WB_READY                  1
#define BLE_OTA_PROTOCOL_WB_EXTENDED               2

#define BLE_OTA_COMMAND_SIZE                       5

#define BLE_OTA_COMMAND_STOP                       0x00
#define BLE_OTA_COMMAND_START_WIRELESS             0x01
#define BLE_OTA_COMMAND_START_APPLICATION          0x02
#define BLE_OTA_COMMAND_FINISH                     0x07
#define BLE_OTA_COMMAND_CANCEL                     0x08

#define BLE_OTA_EVENT_SIZE                         1

#define BLE_OTA_EVENT_REBOOT_CONFIRMED             0x01 // WB
#define BLE_OTA_EVENT_ALLOW                        0x02 // WB_READY "Ready To Receive File"
#define BLE_OTA_EVENT_DENY                         0x03 // WB_READY "Error Not Free"

#define BLE_OTA_EVENT_EXTENDED                     0x80
#define BLE_OTA_EVENT_CANCEL                       0x80 // WB_EXTENDED "Canceled"
#define BLE_OTA_EVENT_ERR_TARGET                   0x81 // WB_EXTENDED "Data not for this device"
#define BLE_OTA_EVENT_ERR_FILE                     0x82 // WB_EXTENDED "Data corrupted"
#define BLE_OTA_EVENT_ERR_WRITE                    0x83 // WB_EXTENDED "Data overflow"
#define BLE_OTA_EVENT_ERR_ERASE                    0x84 // WB_EXTENDED "FLASH erase failure"
#define BLE_OTA_EVENT_ERR_CHECK_ERASED             0x85 // WB_EXTENDED "FLASH erase check failure"
#define BLE_OTA_EVENT_ERR_PROGRAM                  0x86 // WB_EXTENDED "FLASH program failure"
#define BLE_OTA_EVENT_ERR_VERIFY                   0x87 // WB_EXTENDED "FLASH program check failure"
#define BLE_OTA_EVENT_ERR_ADDRESS                  0x88 // WB_EXTENDED "FLASH program address out of bounds"
#define BLE_OTA_EVENT_ERR_INTERNAL                 0x8a // WB_EXTENDED "Internal Error"

#define BLE_OTA_DATA_SIZE                          244

#define BLE_OTA_COUNT_SIZE                         2

#define BLE_OTA_STATE_BEGIN                        0
#define BLE_OTA_STATE_STOP                         1
#define BLE_OTA_STATE_START                        2
#define BLE_OTA_STATE_FINISH                       3
#define BLE_OTA_STATE_REBOOT                       4

#define BLE_OTA_PROGRAM_SIZE                       1024

#define BLE_OTA_SEQUENCE_ACCEPT                     0x01
#define BLE_OTA_SEQUENCE_REJECT                     0x02
#define BLE_OTA_SEQUENCE_BEGIN                      0x04
#define BLE_OTA_SEQUENCE_START                      0x08
#define BLE_OTA_SEQUENCE_FINISH                     0x10
#define BLE_OTA_SEQUENCE_CONFIRM                    0x20
#define BLE_OTA_SEQUENCE_CANCEL                     0x40

typedef struct __attribute__((packed)) _ble_ota_command_t {
    uint8_t command;
    uint8_t address[3];
    uint8_t sectors;
} ble_ota_command_t;

typedef struct __attribute__((packed)) _ble_ota_event_t {
    uint8_t event;
} ble_ota_event_t;

typedef struct __attribute__((packed)) _ble_ota_count_t {
    uint16_t count;
} ble_ota_count_t;

BLEOta::BLEOta() :
    BLEService("0000fe20-cc7a-482a-984a-7f2ed5b3e58f")
{
    m_busy = false;
    m_state = BLE_OTA_STATE_BEGIN;
    m_protocol = BLE_OTA_PROTOCOL_WB;
    m_component = STM32WB_FWU_COMPONENT_NONE;
    m_deny = 2;
    m_options = 0;
    m_sequence = 0;

    m_head = 0;
    m_tail = 0;
    m_offset = 0;
    m_size = 0;

    m_work = K_WORK_INIT(BLEOta::processCallback, (void*)this);

    m_command_characteristic = BLECharacteristic("0000fe22-8e22-4541-9d4c-21edae82ed19", (BLE_PROPERTY_WRITE_WITHOUT_RESPONSE), 0, BLE_OTA_COMMAND_SIZE, false);
    m_event_characteristic = BLECharacteristic("0000fe23-8e22-4541-9d4c-21edae82ed19", (BLE_PROPERTY_INDICATE), 0, BLE_OTA_EVENT_SIZE, true);
    m_data_characteristic = BLECharacteristic("0000fe24-8e22-4541-9d4c-21edae82ed19", (BLE_PROPERTY_READ | BLE_PROPERTY_WRITE_WITHOUT_RESPONSE), 0, BLE_OTA_DATA_SIZE, false);

    m_start_callback = Callback();
    m_finish_callback = Callback();
    
    m_command_characteristic.onWrite(Callback(&BLEOta::commandCallback, this));
    m_data_characteristic.onWrite(Callback(&BLEOta::dataCallback, this));
    m_data_characteristic.onRead(Callback(&BLEOta::countCallback, this));
    
    addCharacteristic(m_command_characteristic);
    addCharacteristic(m_event_characteristic);
    addCharacteristic(m_data_characteristic);

#if 0    
    onConnect(Callback(&BLEOta::connectCallback, this));
    onDisconnect(Callback(&BLEOta::disconnectCallback, this));
#endif    
}

bool BLEOta::begin(uint32_t options) {
    if (m_state != BLE_OTA_STATE_BEGIN) {
        return false;
    }

    m_options = options;

    armv7m_atomic_orb(&m_sequence, BLE_OTA_SEQUENCE_BEGIN);

    k_work_submit(&m_work);
    
    return true;
}

uint32_t BLEOta::state() {
    uint32_t state = stm32wb_fwu_state();

    if (state == STM32WB_FWU_STATE_READY) {
        return BLE_OTA_STATE_READY;
    }

    if (state >= STM32WB_FWU_STATE_FAILED) {
        return state - (STM32WB_FWU_STATE_FAILED - BLE_OTA_STATE_FAILED);
    }
    
    return BLE_OTA_STATE_BUSY;
}

void BLEOta::accept() {
    if (m_state == BLE_OTA_STATE_BEGIN) {
        armv7m_atomic_orb(&m_sequence, BLE_OTA_SEQUENCE_ACCEPT);

        k_work_submit(&m_work);
    }
}

void BLEOta::reject() {
    if (m_state == BLE_OTA_STATE_BEGIN) {
        armv7m_atomic_orb(&m_sequence, BLE_OTA_SEQUENCE_ACCEPT);

        k_work_submit(&m_work);
    }
}

void BLEOta::allow() {
    __armv7m_atomic_andb(&m_deny, ~1);
}

void BLEOta::deny() {
    __armv7m_atomic_orb(&m_deny, 1);
}

void BLEOta::confirm() {
    if (m_state == BLE_OTA_STATE_FINISH) {
        armv7m_atomic_orb(&m_sequence, BLE_OTA_SEQUENCE_CONFIRM);

        k_work_submit(&m_work);
    }
}

void BLEOta::cancel() {
    if (m_state == BLE_OTA_STATE_FINISH) {
        armv7m_atomic_orb(&m_sequence, BLE_OTA_SEQUENCE_CANCEL);

        k_work_submit(&m_work);
    }
}

void BLEOta::onStart(void(*callback)(void)) {
    m_start_callback = Callback(callback);
}

void BLEOta::onStart(Callback callback) {
    m_start_callback = callback;
}

void BLEOta::onFinish(void(*callback)(void)) {
    m_finish_callback = Callback(callback);
}

void BLEOta::onFinish(Callback callback) {
    m_finish_callback = callback;
}

void BLEOta::acceptCallback(class BLEOta *self) {
#if (BLE_TRACE_SUPPORTED == 1)
    armv7m_rtt_printf("ACCEPT\n");
#endif

    self->m_busy = false;

    k_work_submit(&self->m_work);
}

void BLEOta::rejectCallback(class BLEOta *self) {
#if (BLE_TRACE_SUPPORTED == 1)
    armv7m_rtt_printf("REJECT\n");
#endif

    self->m_busy = false;

    k_work_submit(&self->m_work);
}

void BLEOta::beginCallback(class BLEOta *self) {
#if (BLE_TRACE_SUPPORTED == 1)
    armv7m_rtt_printf("BEGIN\n");
#endif

    __armv7m_atomic_andb(&self->m_deny, ~2);
    
    self->m_state = BLE_OTA_STATE_STOP;

    self->m_busy = false;
    
    k_work_submit(&self->m_work);
}

void BLEOta::startCallback(class BLEOta *self) {
    ble_ota_event_t event;
    
#if (BLE_TRACE_SUPPORTED == 1)
    armv7m_rtt_printf("START(%d)\n", self->m_request.status);
#endif

    self->m_status = 0;
    
    if (self->m_request.status == STM32WB_FWU_STATUS_NO_ERROR) {
#if (BLE_TRACE_SUPPORTED == 1)
        armv7m_rtt_printf("### ALLOW ###\n");
#endif

        if (self->m_protocol >= BLE_OTA_PROTOCOL_WB_READY) {
            event.event = BLE_OTA_EVENT_ALLOW;
            
            self->m_event_characteristic.notifyValue(&event, BLE_OTA_EVENT_SIZE);
        }
    } else {
#if (BLE_TRACE_SUPPORTED == 1)
        armv7m_rtt_printf("### DENY ###\n");
#endif

        if (!self->m_status) {
            self->m_status = BLE_OTA_EVENT_EXTENDED + self->m_request.status;
        }
        armv7m_atomic_orb(&self->m_sequence, BLE_OTA_SEQUENCE_CANCEL);

        if (self->m_protocol >= BLE_OTA_PROTOCOL_WB_READY) {
            event.event = BLE_OTA_EVENT_DENY;
            
            self->m_event_characteristic.notifyValue(&event, BLE_OTA_EVENT_SIZE);
        }
    }

    self->m_busy = false;
    
    k_work_submit(&self->m_work);
}

void BLEOta::writeCallback(class BLEOta *self) {
    if (self->m_request.status == STM32WB_FWU_STATUS_NO_ERROR) {
        self->m_head += self->m_size;

        if (self->m_head >= BLE_OTA_QUEUE_SIZE) {
            self->m_head -= BLE_OTA_QUEUE_SIZE;
        }
        
        self->m_offset += self->m_size;
        self->m_size = 0;
    
#if (BLE_TRACE_SUPPORTED == 1)
        {
            uint32_t head, tail, level;
            
            head = self->m_head;
            tail = self->m_tail;
            
            if (tail >= head) {
                level = tail - head;
            } else {
                level = (BLE_OTA_QUEUE_SIZE - head) + tail;
            }
            
            armv7m_rtt_printf("WRITE(%d)\n", (BLE_OTA_QUEUE_SIZE - level - 1));
        }
#endif
    } else {
        if (!self->m_status) {
            self->m_status = BLE_OTA_EVENT_EXTENDED + self->m_request.status;
        }
        armv7m_atomic_orb(&self->m_sequence, BLE_OTA_SEQUENCE_CANCEL);
    }
    
    self->m_busy = false;

    k_work_submit(&self->m_work);
}

void BLEOta::finishCallback(class BLEOta *self) {
#if (BLE_TRACE_SUPPORTED == 1)
    armv7m_rtt_printf("FINISH (%d)\n", self->m_request.status);
#endif

    if (self->m_request.status == STM32WB_FWU_STATUS_NO_ERROR) {
        self->m_state = BLE_OTA_STATE_FINISH;

        self->m_finish_callback();

        if (!(self->m_options & BLE_OTA_OPTION_CANDIDATE)) {
            armv7m_atomic_orb(&self->m_sequence, BLE_OTA_SEQUENCE_CONFIRM);
        }
    } else {
        if (!self->m_status) {
            self->m_status = BLE_OTA_EVENT_EXTENDED + self->m_request.status;
        }
        armv7m_atomic_orb(&self->m_sequence, BLE_OTA_SEQUENCE_CANCEL);
    }
    
    self->m_busy = false;

    k_work_submit(&self->m_work);
}

void BLEOta::confirmCallback(class BLEOta *self) {
    ble_ota_event_t event;
    
#if (BLE_TRACE_SUPPORTED == 1)
    armv7m_rtt_printf("CONFIRM (%d)\n", self->m_request.status);
#endif

    if (self->m_request.status == STM32WB_FWU_STATUS_NO_ERROR) {
        self->m_state = BLE_OTA_STATE_REBOOT;

        event.event = BLE_OTA_EVENT_REBOOT_CONFIRMED;
            
        self->m_event_characteristic.notifyValue(&event, BLE_OTA_EVENT_SIZE, BLEOta::rebootCallback);
    } else {
        if (!self->m_status) {
            self->m_status = BLE_OTA_EVENT_EXTENDED + self->m_request.status;
        }
        armv7m_atomic_orb(&self->m_sequence, BLE_OTA_SEQUENCE_CANCEL);
    }
    
    self->m_busy = false;

    k_work_submit(&self->m_work);
}

void BLEOta::rebootCallback() {
    stm32wb_usbd_disable();
    
    stm32wb_system_reset();
}

void BLEOta::cancelCallback(class BLEOta *self) {
    ble_ota_event_t event;
    
#if (BLE_TRACE_SUPPORTED == 1)
    armv7m_rtt_printf("CANCEL\n");
#endif

    if (self->m_protocol >= BLE_OTA_PROTOCOL_WB_EXTENDED) {
        event.event = self->m_status ? self->m_status : BLE_OTA_EVENT_CANCEL;
        
        self->m_event_characteristic.notifyValue(&event, BLE_OTA_EVENT_SIZE);
    }
    
    self->m_state = BLE_OTA_STATE_STOP;
    self->m_status = 0;
    self->m_protocol = BLE_OTA_PROTOCOL_WB;
    self->m_head = 0;
    self->m_tail = 0;
    self->m_offset = 0;
    self->m_size = 0;
    
    self->m_busy = false;

    k_work_submit(&self->m_work);
}

void BLEOta::processCallback(class BLEOta *self) {
    uint32_t fwu_state, head, tail, level, size, sequence;
    ble_ota_event_t event;
    
    if (self->m_busy) {
        return;
    }

    fwu_state = stm32wb_fwu_state();
    
    sequence = self->m_sequence;

    if (sequence & BLE_OTA_SEQUENCE_ACCEPT) {
        armv7m_atomic_andb(&self->m_sequence, ~BLE_OTA_SEQUENCE_ACCEPT);

        self->m_busy = true;
                
        self->m_request.control = STM32WB_FWU_CONTROL_ACCEPT;
        self->m_request.offset = 0;
        self->m_request.data = NULL;
        self->m_request.size = 0;
        self->m_request.callback = (stm32wb_fwu_done_callback_t)BLEOta::acceptCallback;
        self->m_request.context = self;
        
        if (stm32wb_fwu_request(&self->m_request)) {
            return;
        }
        
        self->m_busy = false;
    }

    if (sequence & BLE_OTA_SEQUENCE_REJECT) {
        armv7m_atomic_andb(&self->m_sequence, ~BLE_OTA_SEQUENCE_REJECT);

        self->m_busy = true;
                
        self->m_request.control = STM32WB_FWU_CONTROL_REJECT;
        self->m_request.offset = 0;
        self->m_request.data = NULL;
        self->m_request.size = 0;
        self->m_request.callback = (stm32wb_fwu_done_callback_t)BLEOta::rejectCallback;
        self->m_request.context = self;
        
        if (stm32wb_fwu_request(&self->m_request)) {
            return;
        }
        
        self->m_busy = false;
    }

    if (sequence & BLE_OTA_SEQUENCE_BEGIN) {
        armv7m_atomic_andb(&self->m_sequence, ~BLE_OTA_SEQUENCE_BEGIN);

        if (fwu_state == STM32WB_FWU_STATE_READY) {
            __armv7m_atomic_andb(&self->m_deny, ~2);

            self->m_state = BLE_OTA_STATE_STOP;
        } else {
            self->m_busy = true;
            
            self->m_request.control = STM32WB_FWU_CONTROL_CLEAN;
            
            if (fwu_state == STM32WB_FWU_STATE_TRIAL) {
                self->m_request.control |= STM32WB_FWU_CONTROL_ACCEPT;
            } else {
                if ((fwu_state != STM32WB_FWU_STATE_FAILED) && (fwu_state != STM32WB_FWU_STATE_UPDATED)) {
                    self->m_request.control |= STM32WB_FWU_CONTROL_CANCEL;
                }
            }
            
            self->m_request.offset = 0;
            self->m_request.data = NULL;
            self->m_request.size = 0;
            self->m_request.callback = (stm32wb_fwu_done_callback_t)BLEOta::beginCallback;
            self->m_request.context = self;
            
            if (stm32wb_fwu_request(&self->m_request)) {
                return;
            }
            
            self->m_busy = false;
        }
    }
        
    if (sequence & BLE_OTA_SEQUENCE_START) {
        armv7m_atomic_andb(&self->m_sequence, ~BLE_OTA_SEQUENCE_START);

        self->m_busy = true;

        self->m_request.control = STM32WB_FWU_CONTROL_START;
        self->m_request.component = self->m_component;
        self->m_request.offset = 0;
        self->m_request.data = NULL;
        self->m_request.size = 0;
        self->m_request.callback = (stm32wb_fwu_done_callback_t)BLEOta::startCallback;
        self->m_request.context = self;

        if (stm32wb_fwu_request(&self->m_request)) {
            return;
        }

#if (BLE_TRACE_SUPPORTED == 1)
        armv7m_rtt_printf("### DENY ###\n");
#endif

        if (self->m_protocol >= BLE_OTA_PROTOCOL_WB_READY) {
            event.event = BLE_OTA_EVENT_DENY;
            
            self->m_event_characteristic.notifyValue(&event, BLE_OTA_EVENT_SIZE);
        }
        
        self->m_busy = false;
    }

    if ((fwu_state == STM32WB_FWU_STATE_WRITING) && !(sequence & BLE_OTA_SEQUENCE_CANCEL)) { 
        head = self->m_head;
        tail = self->m_tail;
        
        if (head != tail) {
            if (tail >= head) {
                level = tail - head;
            } else {
                level = (BLE_OTA_QUEUE_SIZE - head) + tail;
            }

            if ((level >= BLE_OTA_PROGRAM_SIZE) || (sequence & BLE_OTA_SEQUENCE_FINISH)) {
                size = BLE_OTA_PROGRAM_SIZE;
                if (size > level) {
                    size = level;
                }
                
#if (BLE_TRACE_SUPPORTED == 1)
                armv7m_rtt_printf("DEQUEUE(%d) [%08x/%d]\n", (BLE_OTA_QUEUE_SIZE - level -1), self->m_offset, ((size + 7) & ~7));
#endif                
                self->m_busy = true;

                self->m_size = size;
                
                self->m_request.control = STM32WB_FWU_CONTROL_WRITE;
                self->m_request.offset = self->m_offset;
                self->m_request.data = &self->m_data[self->m_head];
                self->m_request.size = ((size + 7) & ~7);
                self->m_request.callback = (stm32wb_fwu_done_callback_t)BLEOta::writeCallback;
                self->m_request.context = self;
                
                if (stm32wb_fwu_request(&self->m_request)) {
                    return;
                }

                self->m_busy = false;

                if (!self->m_status) {
                    self->m_status = BLE_OTA_EVENT_ERR_INTERNAL;
                }
                
                sequence |= BLE_OTA_SEQUENCE_CANCEL;
            }
        }
    }

    if ((sequence & BLE_OTA_SEQUENCE_FINISH) && !(sequence & BLE_OTA_SEQUENCE_CANCEL)) {
        armv7m_atomic_andb(&self->m_sequence, ~BLE_OTA_SEQUENCE_FINISH);

        self->m_busy = true;

        self->m_request.control = STM32WB_FWU_CONTROL_FINISH;
        self->m_request.offset = 0;
        self->m_request.data = NULL;
        self->m_request.size = 0;
        self->m_request.callback = (stm32wb_fwu_done_callback_t)BLEOta::finishCallback;
        self->m_request.context = self;

        if (stm32wb_fwu_request(&self->m_request)) {
            return;
        }

        self->m_busy = true;
    }

    if ((sequence & BLE_OTA_SEQUENCE_CONFIRM) && !(sequence & BLE_OTA_SEQUENCE_CANCEL)) {
        armv7m_atomic_andb(&self->m_sequence, ~BLE_OTA_SEQUENCE_CONFIRM);

        self->m_busy = true;

        self->m_request.control = STM32WB_FWU_CONTROL_INSTALL;
        self->m_request.mode = (self->m_options & BLE_OTA_OPTION_TRIAL) ? STM32WB_FWU_MODE_TRIAL : STM32WB_FWU_MODE_ACCEPT;
        self->m_request.offset = 0;
        self->m_request.data = NULL;
        self->m_request.size = 0;
        self->m_request.callback = (stm32wb_fwu_done_callback_t)BLEOta::confirmCallback;
        self->m_request.context = self;

        if (stm32wb_fwu_request(&self->m_request)) {
            return;
        }

        self->m_busy = true;
    }

    if (sequence & BLE_OTA_SEQUENCE_CANCEL) {
        self->m_sequence = sequence = 0;

        self->m_busy = true;

        self->m_request.control = STM32WB_FWU_CONTROL_CANCEL | STM32WB_FWU_CONTROL_CLEAN;
        self->m_request.offset = 0;
        self->m_request.data = NULL;
        self->m_request.size = 0;
        self->m_request.callback = (stm32wb_fwu_done_callback_t)BLEOta::cancelCallback;
        self->m_request.context = self;

        if (stm32wb_fwu_request(&self->m_request)) {
            return;
        }

        self->m_busy = true;
        self->m_protocol = BLE_OTA_PROTOCOL_WB;
    }
}

void BLEOta::connectCallback() {
#if (BLE_TRACE_SUPPORTED == 1)
    armv7m_rtt_printf("CONNECT\n");
#endif
}

void BLEOta::disconnectCallback() {
#if (BLE_TRACE_SUPPORTED == 1)
    armv7m_rtt_printf("DISCONNECT\n");
#endif

    if (stm32wb_fwu_state() != STM32WB_FWU_STATE_READY) {
        armv7m_atomic_orb(&m_sequence, BLE_OTA_SEQUENCE_CANCEL);

        k_work_submit(&m_work);
    }
}

void BLEOta::commandCallback() {
    uint32_t length;
    uint8_t data[BLE_OTA_COMMAND_SIZE];
    ble_ota_event_t event;
    
    if (m_command_characteristic.written()) {
      length = m_command_characteristic.getValue(&data[0], sizeof(data));

        if (length) {
            switch (data[0]) {
            case BLE_OTA_COMMAND_STOP: {
#if (BLE_TRACE_SUPPORTED == 1)
                armv7m_rtt_printf("STOP\n");
#endif
                break;
            }
                
            case BLE_OTA_COMMAND_START_WIRELESS: {
#if (BLE_TRACE_SUPPORTED == 1)
                armv7m_rtt_printf("START_WIRELESS\n");
#endif

                if (length == 5) {
                    m_protocol = (data[4] == 0xff) ? BLE_OTA_PROTOCOL_WB_EXTENDED : BLE_OTA_PROTOCOL_WB_READY;
                }

                if (m_options & BLE_OTA_OPTION_WIRELESS) {
                    if (m_state == BLE_OTA_STATE_STOP) {
                        m_start_callback();

                        if (!m_deny) {
                            m_state = BLE_OTA_STATE_START;
                            m_component = STM32WB_FWU_COMPONENT_WIRELESS;
                            
                            armv7m_atomic_orb(&m_sequence, BLE_OTA_SEQUENCE_START);
                            
                            k_work_submit(&m_work);
                            
                            return;
                        }
                    }
                }
                
                if (m_protocol >= BLE_OTA_PROTOCOL_WB_READY) {
                    event.event = BLE_OTA_EVENT_DENY;
            
                    m_event_characteristic.notifyValue(&event, BLE_OTA_EVENT_SIZE);
                }
                break;
            }
                
            case BLE_OTA_COMMAND_START_APPLICATION: {
#if (BLE_TRACE_SUPPORTED == 1)
              armv7m_rtt_printf("START_APPLICATION %02x %02x %02x %02x %02x\n", data[0], data[1], data[2], data[3], data[4]);
#endif

                if (length == 5) {
                    m_protocol = (data[4] == 0xff) ? BLE_OTA_PROTOCOL_WB_EXTENDED : BLE_OTA_PROTOCOL_WB_READY;
                }

                if (m_state == BLE_OTA_STATE_STOP) {
                    m_start_callback();

                    if (!m_deny) {
                        m_state = BLE_OTA_STATE_START;
                        m_component = STM32WB_FWU_COMPONENT_APPLICATION;
                        
                        armv7m_atomic_orb(&m_sequence, BLE_OTA_SEQUENCE_START);
                        
                        k_work_submit(&m_work);
                        
                        return;
                    }
                }
                
                if (m_protocol >= BLE_OTA_PROTOCOL_WB_READY) {
                    event.event = BLE_OTA_EVENT_DENY;
            
                    m_event_characteristic.notifyValue(&event, BLE_OTA_EVENT_SIZE);
                }
                break;
            }
            
            case BLE_OTA_COMMAND_FINISH: {
#if (BLE_TRACE_SUPPORTED == 1)
                armv7m_rtt_printf("FINISH\n");
#endif

                if (m_state == BLE_OTA_STATE_START) {
                    armv7m_atomic_orb(&m_sequence, BLE_OTA_SEQUENCE_FINISH);

                    k_work_submit(&m_work);

                    return;
                }
                break;
            }
            
            case BLE_OTA_COMMAND_CANCEL: {
#if (BLE_TRACE_SUPPORTED == 1)
                armv7m_rtt_printf("CANCEL\n");
#endif

                if ((m_state == BLE_OTA_STATE_START) || (m_state == BLE_OTA_STATE_FINISH)) {
                    armv7m_atomic_orb(&m_sequence, BLE_OTA_SEQUENCE_CANCEL);

                    k_work_submit(&m_work);

                    return;
                }
                break;
            }

            default:
#if (BLE_TRACE_SUPPORTED == 1)
                armv7m_rtt_printf("UNKNOWN %02x (%d)\n", data[0], length);
#endif                
                break;
            }
        }
    }
}

void BLEOta::dataCallback() {
    uint32_t head, tail, level, count, size;
    uint8_t data[BLE_OTA_DATA_SIZE];
    
    if (m_data_characteristic.written()) {
        size = m_data_characteristic.getValue(&data[0], BLE_OTA_DATA_SIZE);

        if (m_state == BLE_OTA_STATE_START) {
            head = m_head;
            tail = m_tail;

            if (tail >= head) {
                level = tail - head;
            } else {
                level = (BLE_OTA_QUEUE_SIZE - head) + tail;
            }
            
            if ((BLE_OTA_QUEUE_SIZE - level - 1) < size) {
#if (BLE_TRACE_SUPPORTED == 1)
                armv7m_rtt_printf("### OVERFLOW ### (%d %d)\n", (BLE_OTA_QUEUE_SIZE - level - 1), size);
#endif

                if (!m_status) {
                    m_status = BLE_OTA_EVENT_ERR_WRITE;
                }
                armv7m_atomic_orb(&m_sequence, BLE_OTA_SEQUENCE_CANCEL);

                k_work_submit(&m_work);
                
                return;
            }

            if (size) {
                count = BLE_OTA_QUEUE_SIZE - tail;
                
                if (count > size) {
                    count = size;
                }
                
                if (count) {
                    memcpy(&m_data[tail], &data[0], count);
                }
                
                if (size != count) {
                    memcpy(&m_data[0], &data[count], (size - count));
                }
                
                tail += size;
                
                if (tail >= BLE_OTA_QUEUE_SIZE) {
                    tail -= BLE_OTA_QUEUE_SIZE;
                }
                
                m_tail = tail;
                
#if (BLE_TRACE_SUPPORTED == 1)
                armv7m_rtt_printf("ENQUEUE(%d) -> %d\n", (BLE_OTA_QUEUE_SIZE - level - 1), size);
#endif
                
                k_work_submit(&m_work);
            }
        }
    }
}

void BLEOta::countCallback() {
    uint32_t head, tail, level;
    ble_ota_count_t count;

    head = m_head;
    tail = m_tail;
            
    if (tail >= head) {
        level = tail - head;
    } else {
        level = (BLE_OTA_QUEUE_SIZE - head) + tail;
    }

    count.count = (BLE_OTA_QUEUE_SIZE - level - 1);

#if (BLE_TRACE_SUPPORTED == 1)
    armv7m_rtt_printf("COUNT %d\n", count.count);
#endif                

    m_data_characteristic.setValue(&count, sizeof(BLE_OTA_COUNT_SIZE));
}

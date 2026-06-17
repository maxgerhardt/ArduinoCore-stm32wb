import asyncio
import sys
from os import path
import platform

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
from bleak.backends.device import BLEDevice

OTA_COMMAND_UUID = "0000fe22-8e22-4541-9d4c-21edae82ed19"
OTA_EVENT_UUID = "0000fe23-8e22-4541-9d4c-21edae82ed19"
OTA_DATA_UUID = "0000fe24-8e22-4541-9d4c-21edae82ed19"

ota_data_size = 20

def handle_disconnect(_: BleakClient):
    print("OTA DISCONNECT")
    
async def main():
    queue = asyncio.Queue()

    async def handle_event(characteristic: BleakGATTCharacteristic, data: bytearray):
        await queue.put(data[0]);
        
    firmware_data = open(sys.argv[1], "rb").read()
    firmware_size = len(firmware_data)
    firmware_offset = 0
        
    device = await BleakScanner.find_device_by_name("STM32WB")

    if device is None:
        print("OTA SETUP -- matching device found.")
        sys.exit(1)

    async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
        if client._backend.__class__.__name__ == "BleakClientBlueZDBus":
            await client._backend._acquire_mtu()
            
        print("OTA CONNECT")
            
        ota_data_size = client.mtu_size - 3
        if ota_data_size > 244:
            ota_data_size = 244
            
        await client.start_notify(OTA_EVENT_UUID, handle_event)

        await client.write_gatt_char(OTA_COMMAND_UUID, bytearray([0x02, 0x00, 0x40, 0x00, 0xff]))
        print("OTA START", ota_data_size)

        try:
            get_await = queue.get()
            event = await asyncio.wait_for(get_await, 10)
        except asyncio.TimeoutError:
            print("OTA TIMEOUT")
            sys.exit(1)

        if event != 2:
            print("OTA DENY, ", event)
            sys.exit(1)

        queue.task_done();
        print("OTA ALLOW")

        chunk_count = 0;

        while firmware_offset < firmware_size:
            chunk_size = ota_data_size
            if chunk_size > (firmware_size - firmware_offset):
                chunk_size = (firmware_size - firmware_offset)
                                            
            if chunk_count < chunk_size:
                count_data = await client.read_gatt_char(OTA_DATA_UUID)
                chunk_count = (count_data[0] << 0) | (count_data[1] << 8)
        
                while chunk_count < chunk_size:
                    await asyncio.sleep(0.025)
                    count_data = await client.read_gatt_char(OTA_DATA_UUID)
                    chunk_count = (count_data[0] << 0) | (count_data[1] << 8)
                    print(chunk_count);

                print("OTA DATA", firmware_offset, chunk_count)
                
            chunk_start = firmware_offset
            chunk_end = firmware_offset + chunk_size
   
            await client.write_gatt_char(OTA_DATA_UUID, firmware_data[chunk_start:chunk_end])

            firmware_offset += chunk_size
            chunk_count -= chunk_size

        await client.write_gatt_char(OTA_COMMAND_UUID, bytearray([0x07]))
        print("OTA_FINISH")

        try:
            get_await = queue.get()
            event = await asyncio.wait_for(get_await, 30)
        except asyncio.TimeoutError:
            print("OTA TIMEOUT")
            sys.exit(1)

        if event != 1:
            print("OTA FAILURE", event)
            sys.exit(1)

        queue.task_done();
        print("OTA REBOOT")
            
if __name__ == "__main__":
    if path.exists(sys.argv[1]):
        asyncio.run(main())
    else:
        print("OTA USAGE: python3 stm32ota.py \"firmware.ota\"")
                
        

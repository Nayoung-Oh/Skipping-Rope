# code structure from https://blog.naver.com/chandong83/222308299341?&isInf=true
import asyncio
from bleak import BleakClient
import struct
import csv

# In this case, we just used fixed address
address = "35:98:c6:4f:26:73"
uuid_list = [
    "3391f4fc-0293-11ee-be56-0242ac120002",
]
uuid_count = "339204c4-0293-11ee-be56-0242ac120002"

tmp_save = []
sensor_data = [0, 0, 0, 0, 0, 0, 0]
def notify_callback_count(sender: int, data: bytearray):
    # in the case of count, just show the counting number
    res = struct.unpack('H', data)[0]
    print('sender: ', sender.uuid, 'data: ', res)

def notify_callback(sender: int, data: bytearray):
    global tmp_save
    global sensor_data
    # unpack the data and recover the sensor values
    tmp_save.append(sensor_data.copy())
    timestamp = struct.unpack('>H', data[:2])[0]
    sensor_data[0] = timestamp
    for i in range(1, 4):
        value = struct.unpack('>h', data[i*2:i*2+2])[0]/100
        sensor_data[i] = value
    for i in range(3):
        value = struct.unpack('>i', data[i*4+8:i*4+12])[0]/100
        sensor_data[i+4] = value    

async def run(address):              
    async with BleakClient(address, timeout=5.0) as client:                    
        print('connected')
        services = await client.get_services()
        for service in services:
            for characteristic in service.characteristics:
                if characteristic.uuid in uuid_list:  
                    # use notify for faster communication
                    if 'notify' in characteristic.properties:
                        print('try to activate notify.')
                        await client.start_notify(characteristic, notify_callback)
                if characteristic.uuid == uuid_count:
                    if 'notify' in characteristic.properties:
                        await client.start_notify(characteristic, notify_callback_count)

        if client.is_connected:
            print("try to wait")
            # wait for 15 seconds to record data
            await asyncio.sleep(15) 
            print('try to deactivate notify.')
            for uuid in uuid_list:
                await client.stop_notify(uuid)
            await client.stop_notify(uuid_count)
    print('disconnect')

loop = asyncio.get_event_loop()
loop.run_until_complete(run(address))
print('done')

# save the data into csv format
with open("data.csv", "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerows(tmp_save)

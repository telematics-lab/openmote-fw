from influxdb import InfluxDBClient
from datetime import datetime
import influxdb
import logging
import struct
import serial
import sys
import re

try:
    serialPort = serial.Serial(port=f'/dev/ttyUSB1', baudrate=115200)
except serial.SerialException as e:
    logging.error(e)
    print("Errore rilevato...")
    print(e)
    exit()


def storeData(serialMessage: bytes, i):        
    reader_string = str(serialMessage, 'latin1')
    reader_string = re.split(r'\t+', reader_string.rstrip('\t'))
    #print(reader_string)
    
    if len(reader_string) == 9:
        i = i + 1
        id          = reader_string[1]
        temp        = reader_string[3]
        humidity    = reader_string[5]
        pressure    = reader_string[7]

        data = [
            {   
                "measurement": "transmissionData",
                "tags": {
                    "openmoteID": id,                    
                },"fields": {
                    "temp": temp,
                    "humidity": humidity,
                    "pressure": pressure
                }

            }
        ]    
        print("JSON: ", data)

        done = clientTest.write_points(data, protocol="json")
    elif len(reader_string) == 3:
        print("INFO CANALE -> RSSI:", reader_string[1], '\n\n')
    else: 
        print("[ERR] Collisione...\n")
    return i

clientTest = InfluxDBClient(
    host='127.0.0.1', port=8086, database='openmoteTest')

message = bytes(37)
buffer = bytes(1)

i = 0
while True:
    
    try:
        
        buffer += serialPort.read(1)        
                
        if buffer[-1] == 126:
            message = buffer
            buffer = bytes(1)
            if len(message) > 2:                               
                i = storeData(message, i)                
    except Exception as e:
        logging.error(e)
        print(e) 


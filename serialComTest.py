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
    
    if len(reader_string) >= 13:
        i = i + 1
        id          = reader_string[1]
        room        = reader_string[3]
        temp        = float(reader_string[5])
        humidity    = float(reader_string[7])
        pressure    = float(reader_string[9])
        light       = float(reader_string[11])

        data = [
            {   
                "measurement": "transmissionData",
                "tags": {
                    "openmoteID": id,  
                    "room": room
                },"fields": {
                    "temp": temp,
                    "humidity": humidity,
                    "pressure": pressure,
                    "light": light
                }

            }
        ]    
        print("JSON: ", data)

        done = clientTest.write_points(data, protocol="json")
    elif len(reader_string) == 3:
        print("INFO CANALE -> RSSI:", reader_string[1], '\n\n')
    else: 
        print("[ERR] Collisione o pacchetto danneggiato...\n")
        #print(reader_string)
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


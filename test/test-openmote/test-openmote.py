#!/usr/bin/python

'''
@file       test-openmote.py
@author     Pere Tuset-Peiro  (peretuset@openmote.com)
@version    v0.1
@date       May, 2015
@brief      

@copyright  Copyright 2015, OpenMote Technologies, S.L.
            This file is licensed under the GNU General Public License v2.
'''

import serial
import struct
import os

def program():
    ser = serial.Serial(port     = '/dev/ttyUSB1',
                        baudrate = 115200,
                        parity   = serial.PARITY_NONE,
                        stopbits = serial.STOPBITS_ONE,
                        bytesize = serial.EIGHTBITS,
                        xonxoff  = False,
                        rtscts   = False,
                        dsrdtr   = False)
    ser.flushInput()
    
    while(True):
        buff = ser.read(4)  
        
        temp_raw, humi_raw = struct.unpack("<2H", buff)
        
        temperature = -46.85
        temperature += 175.72 * (temp_raw / 65535.0)
        
        humidity = -6.0
        humidity += 125.0 * (humi_raw / 65535.0);

        print temperature, humidity


def main():
    program()
    
if __name__ == "__main__":
    main()

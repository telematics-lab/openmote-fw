#!/usr/bin/python

'''
@file       test-radio.py
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
    print("Checking the RSSI...")

    ser = serial.Serial(port     = '/dev/ttyUSB0',
                    baudrate = 115200)

    while(True):
        buff = ser.read(2);
        data = struct.unpack("bB", buff)
        
        rssi    = data[1]
        crc     = data[0]

        print rssi, crc

def main():
    program()
    
if __name__ == "__main__":
    main()

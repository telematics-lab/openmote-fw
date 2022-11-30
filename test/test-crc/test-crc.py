#!/usr/bin/python

'''
@file       test-crc.py
@author     Pere Tuset-Peiro  (peretuset@openmote.com)
@version    v0.1
@date       February, 2019
@brief      

@copyright  Copyright 2019, OpenMote Technologies, S.L.
            This file is licensed under the GNU General Public License v2.
'''

import os
import sys
pwd = os.path.abspath(__file__)
pwd = os.path.dirname(os.path.dirname(os.path.dirname(pwd)))
pwd = os.path.join(pwd, 'python')
sys.path.append(pwd)

import argparse
import signal
import struct
import time
import random
import string

import Crc16

finished = False

def signal_handler(sig, frame):
    global finished
    finished = True

def main():
    # Set up SIGINT signal
    signal.signal(signal.SIGINT, signal_handler)

    #test_vector = [0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39]
    test_vector = ['1','2','3','4','5','6','7','8','9']
    crc = Crc16.Crc16()

    for i in test_vector:
        val = ord(i)
        #val = i
        print(val, type(val))
        crc.push(val)

    result = crc.get()
    print(hex(result))
    
if __name__ == "__main__":
    main()

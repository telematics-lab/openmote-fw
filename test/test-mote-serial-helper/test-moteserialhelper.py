# -*- coding: utf-8 -*-
#!/usr/bin/python

'''
@file       test-moteserialhelper.py
@author     Xavi Vilajosana  (xvilajosana@openmote.com)
@version    v0.1
@date       Jan, 2019
@brief      

@copyright  Copyright 2015, OpenMote Technologies, S.L.
            This file is licensed under the GNU General Public License v2.
'''

import os
import sys
pwd = os.path.abspath(__file__)
pwd = os.path.dirname(os.path.dirname(os.path.dirname(pwd)))
pwd = os.path.join(pwd, 'python')
print(pwd)
sys.path.append(pwd)

import logging
import os
import signal
import struct
import sys
import time

from MoteSerialHelper import MoteSerialHelper
from Mote import Mote

logger = logging.getLogger(__name__)

uart_speed = 115200

finished = False

def signal_handler(sig, frame):
    global finished
    finished = True

def program():
    global finished

    # Create Serial manager
    motehelper = MoteSerialHelper()
    motehelper.configureSerial(baudrate = uart_speed)
    motes = motehelper.discoverMotes()
    #motes['COM27'] = Mote(port='COM27', baudrate=uart_speed)
    for mote in motes:
        print(mote.getName())
    #print(motes)
    #motes['COM27'].sendData("hola caracola")

def main():
    # Set-up logging back-end
    logging.basicConfig(level=logging.ERROR)
    signal.signal(signal.SIGINT, signal_handler)
    
    program()
    
if __name__ == "__main__":
    main()

#!/usr/bin/python

'''
@file       test-uart.py
@author     Pere Tuset-Peiro  (peretuset@openmote.com)
@version    v0.1
@date       May, 2015
@brief      

@copyright  Copyright 2015, OpenMote Technologies, S.L.
            This file is licensed under the GNU General Public License v2.
'''

import serial
import signal
import struct
import time
import os

finished = False

def signal_handler(sig, frame):
    global finished
    finished = True

def program():
    global finished

    port     = "COM10"
    baudrate = 2304000
    timeout  = 0.1
    count    = 200
    bits     = 10 # start + byte + stop
    message  = count * b"0123456789"
    length   = len(message)

    # Open serial port
    ser = serial.Serial(port     = port,
                        baudrate = baudrate,
                        parity   = serial.PARITY_NONE,
                        stopbits = serial.STOPBITS_ONE,
                        bytesize = serial.EIGHTBITS,
                        timeout  = timeout)

    print("Starting test with {} bytes/message at {} kbps.".format(length, baudrate))
    
    # Prepare statistics
    total_sent     = 0
    total_received = 0

    # Store start time
    start_time = time.time()

    # Repeat until finish condition
    while (not finished):
        # Write message 
        ser.write(message)

        # Account for sent messages
        total_sent += 1

        # Read message with length
        line = ser.read(length)

        # If we have received a reply
        if (len(line) > 0):
            # If the reply contents match
            if (message.decode() == line.decode()):
                # Account for received messages
                total_received += 1
        
        # Calculate elapsed time
        current_time = time.time()
        elapsed_time = 1000 * (current_time - start_time)

        # If elapsed time
        if elapsed_time > 1000:
            bandwidth = total_received * length * bits / elapsed_time
            print("{} out of {} messages received in the last {:.2f} ms ({:.2f} kbps)".format(total_received, total_sent, elapsed_time, bandwidth))

            # Restore start time
            start_time = current_time

            # Restore statistics
            total_sent = 0
            total_received = 0

def main():
    # Set up SIGINT signal
    signal.signal(signal.SIGINT, signal_handler)

    # Execute program
    program()
    
if __name__ == "__main__":
    main()

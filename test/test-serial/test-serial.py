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

import os
import sys
pwd = os.path.abspath(__file__)
pwd = os.path.dirname(os.path.dirname(os.path.dirname(pwd)))
pwd = os.path.join(pwd, 'python')
sys.path.append(pwd)


import argparse
import random
import signal
import string
import struct
import time

import Serial

finished = False

def signal_handler(sig, frame):
    global finished
    finished = True

def program(port = None, baudrate = None, uncertain = None, length = None, timeout = None, sleep = None, duration = None, report = None):
    global finished

    bits = 10

    # Calculate duration in miliseconds
    if (duration > 0):
        duration = 1000 * duration
    else:
        duration = -1

    # Generate random message
    if (uncertain == True):
        message     = "".join(random.choice(string.ascii_letters) for x in range(length))
    else:
        message     = "".join("a" for x in range(length))
    tx_length = len(message)

    # Create and start Serial manager
    serial = Serial.Serial(name = port, baudrate = baudrate, timeout = timeout)
    serial.start()

    if (uncertain == True):
        print("Starting test at port {} with {} bauds, RANDOM message length {} bytes and {} sleep for {} ms.".format(port, baudrate, tx_length, sleep, duration))
    else:
        print("Starting ping at port {} with {} bauds, message length {} bytes and {} sleep for {} ms.".format(port, baudrate, tx_length, sleep, duration))

    # Prepare statistics
    total_sent       = 0
    total_received   = 0
    partial_sent     = 0
    partial_received = 0

    # Store start time
    start_time = time.time()
    total_time = 0

    # Repeat until finish condition
    while (not finished):
        # Transmit message 
        serial.transmit(message)

        # Account for sent messages
        partial_sent += 1

        # Try to receive a Serial message
        received, length = serial.receive(timeout = timeout)

        # Generate random message
        if (uncertain == True):
            message = "".join(random.choice(string.ascii_letters) for x in range(length))

        # If we received a message
        if (length > 0):
            # Account for received messages
            partial_received += 1
        
        # Calculate elapsed time
        current_time = time.time()
        elapsed_time = 1000 * (current_time - start_time)

        # If elapsed time is above report time
        if (elapsed_time > report):
            # Updated statistics
            total_sent += partial_sent
            total_received += partial_received

            bandwidth = partial_received * length * bits / elapsed_time
            print("{} out of {} messages received in the last {:.2f} ms ({:.2f} kbps)".format(partial_received, partial_sent, elapsed_time, bandwidth))
            print("Received {} and transmitted {} packets.".format(total_received, total_sent))

            # Restore start time
            start_time = current_time

            # Restore statistics
            partial_sent = 0
            partial_received = 0

            # Account for elapsed time
            total_time  += elapsed_time

        # If total time is above total time
        if (duration != -1 and total_time > duration):
            # Signal the script to finish
            finished = True

    # Stop the serial port
    serial.stop()

def main():
    # Set up SIGINT signal
    signal.signal(signal.SIGINT, signal_handler)

    # Create argument parser
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("-p", "--port", type=str, required=True)
    parser.add_argument("-b", "--baudrate", type=int, default=115200)
    parser.add_argument("-l", "--length", type=int, default=100)
    parser.add_argument("-d", "--duration", type=int, default=0)
    parser.add_argument("-r", "--report", type=int, default=1000)
    parser.add_argument("-s", "--sleep", type=float, default=0.1)
    parser.add_argument("-t", "--timeout", type=float, default=1.0)
    parser.add_argument("-u", "--uncertain", type=lambda s: (str(s).lower() in ['true', 'on', 'yes', '1']), default=False)

    # Parse arguments
    args = parser.parse_args()

    # Execute program
    program(port = args.port, baudrate=args.baudrate, uncertain=args.uncertain, length=args.length,
            timeout=args.timeout, sleep=args.sleep, duration=args.duration, report=args.report)
    
if __name__ == "__main__":
    main()

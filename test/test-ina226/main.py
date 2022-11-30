# -*- coding: utf-8 -*-
#!/usr/bin/python

import sys
sys.path.append("../../python")

import argparse
import json
import logging
import signal
import struct
import time

import matplotlib.pyplot as plt
import matplotlib.ticker as plticker

import numpy as np

import Ina226
import MqttClient

logger = logging.getLogger(__name__)

ina226_client = None
ina226_data   = list()

mqtt_client = None
mqtt_topic  = None

abort = False

def signal_handler(signal, frame):
    global ina226_client
    global abort
    abort = True
    ina226_client.stop()

def export_results(data = None, data_export = False, data_path = None, data_name = None):
    # Check data length
    if (len(data) > 0):
        pass
    else:
        logger.error("export_results: empty data list.")

def plot_results(data = None, figure_display = False, figure_save = False, figure_path = None, 
                 figure_name = None, figure_format = None, figure_resolution = None):
    # Check data length
    if (len(data) > 0):
        plt.plot(data)
        plt.xlabel('Time (s)')
        plt.ylabel('Current (mA)')
        plt.grid(True)

        # Save figure
        if (figure_save):
            file = figure_path + figure_name + "." + figure_format
            plt.savefig(file, format = figure_format, dpi=figure_resolution)

        # Show figure
        if (figure_display):
            plt.show()
    else:
        logger.error("plot_results: empty data list.")

def data_callback(data):
    global ina226_data
    global mqtt_client
    global mqtt_topic

    # Add data to INA226 data
    ina226_data.extend(data)

    # Calculate mean and standard deviation
    mean = round(np.mean(data), 3)
    std  = round(np.std(data), 3)
    maximum = round(np.max(data), 3)
    minimum = round(np.min(data), 3)

    # Print average, sstandard deviation, maximum and minimum for last period
    print("Average = {} mA, Std. deviation = {} mA, Maximum = {} mA, Minimum = {} mA".format(mean, std, maximum, minimum))

    # Check if we need to send data via MQTT
    if (mqtt_client != None):
        # Prepare MQTT message with JSON
        mqtt_message = json.dumps({"mean": mean, "std": std, "max": maximum, "min": minimum})

        # Send MQTT message
        mqtt_client.send_message(mqtt_topic, mqtt_message)
    
def main():
    global abort
    global mqtt_client
    global mqtt_topic
    global ina226_client
    global ina226_data

    # Set-up logging back-end
    logging.basicConfig(level=logging.ERROR)

    # Parse arguments
    args = parse_arguments()

    # Recover arguments
    manual              = args.manual
    calibrate           = args.calibrate
    calibrate_timeout   = args.calibrate_timeout
    serial_port         = args.serial_port
    serial_baudrate     = args.serial_baudrate
    data_export         = args.data_export
    data_path           = args.data_path
    data_name           = args.data_name
    figure_display      = args.figure_display
    figure_save         = args.figure_save
    figure_path         = args.figure_path
    figure_name         = args.figure_name
    figure_format       = args.figure_format
    figure_resolution   = args.figure_resolution
    mqtt_export         = args.mqtt_export
    mqtt_server         = args.mqtt_server
    mqtt_port           = args.mqtt_port
    mqtt_topic          = args.mqtt_topic
    measurement_offset  = args.measurement_offset
    measurement_timeout = args.measurement_timeout
    callback_function   = data_callback
    callback_period     = args.callback_period

    # If we need to calibrate, force manual operation
    if (calibrate):
        manual = True

    # Setup logging back-end
    #logging.basicConfig(level=logging.DEBUG)
    logging.basicConfig(filename='main.log', filemode='w', level=logging.DEBUG)

    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Create and initialize INA226
    ina226_client = Ina226.Ina226(serial_port = serial_port,
                                  serial_baudrate = serial_baudrate)
    ina226_client.init()
    ina226_client.configure()

    # Setup MQTT
    if (mqtt_export):
        mqtt_client = MqttClient.MqttClient(address = mqtt_server, port = mqtt_port)
        mqtt_client.start()

    # If calibration is required
    if (calibrate):
        try:
            input("Disconnect power from DUT and press enter to start calibration: ")
        except:
            return

        measurement_offset = ina226_client.measure(calibrate = True, measurement_offset = 0.0, measurement_timeout = calibrate_timeout)
        print("Calibration offset = {} mA".format(measurement_offset))

    # Wait for user input
    if (manual):
        try:
            input("Connect power to DUT and press enter to start measurement: ")
        except:
            return

    print()
    print("Starting measurement with timeout = {} ms, period = {} ms and offset = {} mA.".format(measurement_timeout, callback_period, measurement_offset))
    print()
    print("Wait until measurement completes or press CTRL+C to interrupt it.")
    print()

    # Launch measurement (blocking)
    ina226_client.measure(calibrate = False,
                   measurement_offset = measurement_offset,
                   measurement_timeout = measurement_timeout,
                   callback_function = callback_function,
                   callback_period = callback_period)
    
    # Compute measurement results
    elements = len(ina226_data)
    if (elements > 0):
        # Calculate average and standard deviation
        mean    = round(np.mean(ina226_data), 3)
        std     = round(np.std(ina226_data), 3)
        maximum = round(np.max(ina226_data), 3)
        minimum = round(np.min(ina226_data), 3)

        # Print average and standard deviation
        print("*****")
        print("Average = {} mA, Std. deviation = {} mA, Maximum = {} mA, Minimum = {} mA".format(mean, std, maximum, minimum))
        print("*****")

        # Export results to file
        export_results(data = ina226_data)

        # Plot results and save figure
        plot_results(data = ina226_data,
                     figure_display = figure_display,
                     figure_save = figure_save,
                     figure_path = figure_path,
                     figure_name = figure_name,
                     figure_format = figure_format,
                     figure_resolution = figure_resolution)
    
    # Stop INA226 client
    ina226_client.deinit()

def parse_arguments():
    parser = argparse.ArgumentParser(description='Measure current consumption with Texas Instruments MSP432P4111 Launchpad and IMETER Boosterpack (INA226).')
    
    parser.add_argument('--calibrate', type=str2bool, nargs='?', const=1, default=False, help='Perform a calibration before starting to measure.')
    parser.add_argument('--calibrate_timeout', type=int, nargs='?', const=1, default=1000, help='Define the duration (ms) of the calibration (default = 1000).')

    parser.add_argument('--manual',  type=str2bool, nargs='?', const=1, default=False, help='Start the mesurement with user intervention, i.e. press Enter.')

    parser.add_argument('--serial_port', nargs='?', const=1, default="COM8", help='Selects the serial port interface to communicate with the board.')
    parser.add_argument('--serial_baudrate', type=int, nargs='?', const=1, default=460800, help='Selects the serial port baud rate to communicate with the board.')

    parser.add_argument('--measurement_timeout', type=int, nargs='?', const=1, default=0, help='Define the duration (ms) of the measurement (default = 0).')
    parser.add_argument('--measurement_offset', type=float, nargs='?', const=1, default=0.0, help='Define the offset (uA) of the measurement (default = 0.0).')

    parser.add_argument('--callback_period', type=int, nargs='?', const=1, default=1000, help='Define the callback period (ms) to export data (default = 1000).')

    parser.add_argument('--data_export', type=str2bool, nargs='?', const=1, default=False, help='Activate exporting the data as csv (default = false).')
    parser.add_argument('--data_path', nargs='?', const=1, default="./", help='Selects the path where the csv file will be stored (default = ./).')
    parser.add_argument('--data_name', nargs='?', const=1, default="data", help='Selects the file name of the data file (default = ).')
    parser.add_argument('--data_extension', nargs='?', const=1, default="txt", help='Selects the format extension of the data file (defaultdata = txt).')
    
    parser.add_argument('--figure_display', type=str2bool, nargs='?', const=1, default=False, help='Activate exporting the data using matplotlib (default = false).')
    parser.add_argument('--figure_save', type=str2bool, nargs='?', const=1, default=False, help='Activate saving the matplotlib figure (default = false).')
    parser.add_argument('--figure_path', nargs='?', const=1, default="./", help='Selects the path where the matplotlib figure will be stored (default = ).')
    parser.add_argument('--figure_name', nargs='?', const=1, default="figure", help='Selects the file name of the matplotlib figure (default = ).')
    parser.add_argument('--figure_format', nargs='?', const=1, default="png", help='Selects the format (and extension) of the matplotlib figure (default = png, supported = [jpeg, png, pdf]).')
    parser.add_argument('--figure_resolution', type=int, nargs='?', const=1, default=300, help='Selects the resolution (dpi) of the matplotlib figure (default = 300).')

    parser.add_argument('--mqtt_export', type=str2bool, nargs='?', const=1, default=False, help='Activate exporting data through MQTT (default = false).')
    parser.add_argument('--mqtt_server', nargs='?', const=1, default="127.0.0.1", help='Defines the MQTT server address (default = 127.0.0.1).')
    parser.add_argument('--mqtt_port', type=int, nargs='?', const=1, default=1883, help='Defines the MQTT server port (default = 1883).')
    parser.add_argument('--mqtt_topic', nargs='?', const=1, default="current", help='Defines the MQTT topic (default = current).')
    
    args = parser.parse_args()

    return args

def str2bool(v):
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

if __name__ == "__main__":
    main()

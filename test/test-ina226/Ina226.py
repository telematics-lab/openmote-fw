# -*- coding: utf-8 -*-
#!/usr/bin/python

import logging
import struct
import time

import numpy as np

import Serial

logger = logging.getLogger(__name__)

time_ms = lambda: int(round(time.time() * 1000))

class Ina226:

    # INA226 register definitions
    INA226_CONFIG                   = 0x00
    INA226_SHUNT_VOLTAGE            = 0x01
    INA226_BUS_VOLTAGE              = 0x02
    INA226_POWER                    = 0x03
    INA226_CURRENT                  = 0x04
    INA226_CALIBRATION              = 0x05
    INA226_MASK_ENABLE              = 0x06
    INA226_ALERT_LIMIT              = 0x07
    INA226_MANUFACTURER_ID          = 0xFE
    INA226_DIE_ID                   = 0xFF

    # INA226 average number of bits
    INA226_CONFIG_AVG_BIT_1         = 0 << 9
    INA226_CONFIG_AVG_BIT_4         = 1 << 9
    INA226_CONFIG_AVG_BIT_16        = 2 << 9
    INA226_CONFIG_AVG_BIT_64        = 3 << 9
    INA226_CONFIG_AVG_BIT_128       = 4 << 9
    INA226_CONFIG_AVG_BIT_256       = 5 << 9
    INA226_CONFIG_AVG_BIT_512       = 6 << 9
    INA226_CONFIG_AVG_BIT_1024      = 7 << 9

    # INA226 VSHUNT conversion time
    INA226_CONFIG_CT_VSH_140US      = 0 << 3
    INA226_CONFIG_CT_VSH_204US      = 1 << 3
    INA226_CONFIG_CT_VSH_332US      = 2 << 3
    INA226_CONFIG_CT_VSH_588US      = 3 << 3
    INA226_CONFIG_CT_VSH_1100US     = 4 << 3
    INA226_CONFIG_CT_VSH_2116US     = 5 << 3
    INA226_CONFIG_CT_VSH_4156US     = 6 << 3
    INA226_CONFIG_CT_VSH_8244US     = 7 << 3

    # INA226 VBUS conversion time
    INA226_CONFIG_CT_VBUS_140US     = 0 << 6
    INA226_CONFIG_CT_VBUS_204US     = 1 << 6
    INA226_CONFIG_CT_VBUS_332US     = 2 << 6
    INA226_CONFIG_CT_VBUS_588US     = 3 << 6
    INA226_CONFIG_CT_VBUS_1100US    = 4 << 6
    INA226_CONFIG_CT_VBUS_2116US    = 5 << 6
    INA226_CONFIG_CT_VBUS_4156US    = 6 << 6
    INA226_CONFIG_CT_VBUS_8244US    = 7 << 6

    # INA226 operation mode
    INA226_CONFIG_MODE_SHUTDOWN1    = 0 << 0
    INA226_CONFIG_MODE_SV_TRIG      = 1 << 0
    INA226_CONFIG_MODE_BV_TRIG      = 2 << 0
    INA226_CONFIG_MODE_SBV_TRIG     = 3 << 0
    INA226_CONFIG_MODE_SHUTDOWN2    = 4 << 0
    INA226_CONFIG_MODE_SV_CONT      = 5 << 0
    INA226_CONFIG_MODE_BV_CONT      = 6 << 0
    INA226_CONFIG_MODE_SBV_CONT     = 7 << 0

	# INA226 default configuration
    INA226_CONFIG_DEFAULT           = 0x4000

    # INA226 commands
	INA226_CMD_NONE					= 0x00
    INA226_CMD_START				= 0x01
    INA226_CMD_STOP				    = 0x02

    def __init__(self, serial_port = None, serial_baudrate = 115200):
        self.serial_port = serial_port
        self.serial_baudrate = serial_baudrate

        self.configuration = self.INA226_CONFIG_DEFAULT

        self.finish = False

    def init(self):
        self.serial = Serial.Serial(self.serial_port, self.serial_baudrate)
        self.serial.start()

    def deinit(self):
        self.serial.stop()

    def stop(self):
        self.finish = True

    def configure(self):
        self.configuration |= self.INA226_CONFIG_AVG_BIT_1
        self.configuration |= self.INA226_CONFIG_CT_VSH_140US
        self.configuration |= self.INA226_CONFIG_CT_VBUS_140US
        self.configuration |= self.INA226_CONFIG_MODE_SV_CONT

    def measure(self, calibrate = False, measurement_offset = 0.0, measurement_timeout = 0, 
                callback_function = None, callback_period = 1000):
        # Create empty list to hold data
        measurement = list()

        # Prepare and transmit message to start measurement
        message  = struct.pack("<bIh", self.INA226_CMD_START, self.configuration, measurement_timeout)
        message = [chr(b) for b in message]
        self.serial.transmit(message);

        # Setup timeout
        start_time   = time_ms()
        current_time = time_ms()
        report_time  = time_ms() 

        # Updated elapsed and timeout values
        elapsed_time = current_time - start_time
        timeout_time = report_time - start_time

        # Set as not finished
        finished = False

        # Repeat until finished
        while (not finished):
            # Receive serial message
            message, length = self.serial.receive()

            # Process message payload
            if (length == 2):
                # Recover voltage from message
                data = message[1] << 8 | message[0] << 0

                # Convert data to current
                current = self.__convert_data(data = data, calibrate = calibrate, offset = measurement_offset)

                # Append current to measurement
                if (current < 1000.0):
                    measurement.append(current)

                # Call external callback
                if (callback_function is not None):
                    # If report timeout
                    if (timeout_time > callback_period):
                        # Call callback function
                        callback_function(measurement)

                        # If we are not calibrating
                        if (not calibrate):
                            # Empty measurement list
                            measurement = list()

                        # Update report itme
                        report_time = time_ms()

            # Update timeout
            current_time = time_ms()
            elapsed_time = current_time - start_time
            timeout_time = current_time - report_time

            # Check if we need to finish
            finished = self.finish

            # Check for timeout
            if (measurement_timeout > 0 and elapsed_time >= measurement_timeout):
                finished = True

        # Check if were forced to stop
        if (measurement_timeout == 0):
            # Repeat to ensure success
            for i in range(4):
                # Prepare and transmit message to stop measurement
                message = struct.pack("b", 2)
                message = [chr(b) for b in message]
                self.serial.transmit(message);

                # Allow for the message to be transmitted
                time.sleep(0.125)

        # If we are calibrating
        if (calibrate):
            # Calculate measurement offset
            measurement_offset = round(np.mean(measurement), 3)

        # Prepare and transmit message to start measurement
        message  = struct.pack("<b", self.INA226_CMD_STOP)
        message = [chr(b) for b in message]
        self.serial.transmit(message);

        if (calibrate):
            return measurement_offset
        else:
            return 0

    def __convert_data(self, data = 0, calibrate = False, offset = 0.0):
        # Convert voltage (1LSB = 2.5 uV) to current (mA) as the voltage drop in a resistor (R = 0.1 Ohm)
        current = (data * 2.5 / 0.1) / 1000.0

        # Calculate true current value based on offset
        if (not calibrate):
            result = current - offset
        else:
            result = current

        # Ensure resulting current is positive
        if (current < 0.0):
            current = 0.0

        # Return current in mA with 3 decimals
        return round(result, 3)

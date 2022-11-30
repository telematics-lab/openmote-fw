# -*- coding: utf-8 -*-

# Import Python libraries
import serial
import threading
import time
import logging
import binascii

# Import OpenMote libraries
from Hdlc import Hdlc

# Import logging configuration
logger = logging.getLogger(__name__)

class Serial(threading.Thread):
    
    def __init__(self, name = None, baudrate = None, timeout = 0.1):
        assert name     != None, logger.error("Serial port not defined.")
        assert baudrate != None, logger.error("Serial baudrate not defined.")
        
        logger.info('init: Creating the Serial object.')
        
        # Call constructor
        threading.Thread.__init__(self)
        
        # Terminate thread event
        self.stop_event = threading.Event()
        
        # Serial port
        self.serial_port = None
        self.name        = name
        self.baudrate    = baudrate
        self.timeout     = timeout
                
        # HDLC driver
        self.hdlc = Hdlc()
        
        # Receive variables
        self.receive_buffer    = []
        self.receive_message   = []
        self.receive_condition = threading.Condition()
        self.is_receiving      = False
        self.rx_byte           = ''
        self.last_rx_byte      = ''
        
        # Transmit variables 
        self.transmit_buffer    = []
        self.transmit_message   = []
        self.transmit_condition = threading.Condition()

        # Quality control variables
        self.tx_total_frames = 0
        self.tx_good_frames  = 0
        self.tx_bad_frames   = 0
        self.rx_total_frames = 0
        self.rx_good_frames  = 0
        self.rx_bad_frames   = 0
        
        try:
            logger.info("init: Opening the serial port on {} at {} bps.".format(self.name, self.baudrate))
            # Open the serial port
            self.serial_port = serial.Serial(port = self.name, 
                                             baudrate = self.baudrate,
                                             timeout = self.timeout)
        except:
            logger.error("init: Error while opening the serial port on {}.".format(self.name))
            raise Exception
        else:
            logger.info("init: Serial object created.")
    
    # Runs the thread
    def run(self):
        logger.info("run: Starting the Serial.")
        
        if (self.serial_port == None):
            raise
        
        # Flush the serial input/ouput               
        self.serial_port.flushInput()
        self.serial_port.flushOutput() 
        
        # Execute while thread is alive
        while (not self.stop_event.isSet()):
            rx_bytes = []
            rx_length = 0

            try:
                # Read the number of bytes available
                rx_length = self.serial_port.in_waiting

                # If bytes are available read them
                if (rx_length > 0):
                    logger.debug("run: Read {} bytes from serial port on {}.".format(rx_length, self.serial_port))
                    
                    # Try to receive a byte from the serial port (blocking)
                    rx_bytes = self.serial_port.read(size = rx_length)

            except:
                logger.error("run: Error while receiving from the serial port on {}.".format(self.serial_port))
                # Terminate the thread
                self.stop()
                # Break the loop
                break

            # Iterate over received bytes
            for rx_byte in rx_bytes:
                # Recover byte
                self.rx_byte = rx_byte

                # Start of frame
                if ((not self.is_receiving) and 
                    (self.last_rx_byte == self.hdlc.HDLC_FLAG) and
                    (self.rx_byte != self.hdlc.HDLC_FLAG)):
                    logger.debug("run: Start of HDLC frame.")
                    
                    self.is_receiving = True
                    self.receive_buffer = []
                    self.receive_buffer.append(self.hdlc.HDLC_FLAG)
                    self.receive_buffer.append(self.rx_byte)
                    
                # Middle of HDLC frame
                elif ((self.is_receiving) and
                      (self.rx_byte != self.hdlc.HDLC_FLAG)):
                    logger.debug("run: Middle of HDLC frame.")
                    
                    self.receive_buffer.append(self.rx_byte)
                    
                # End of HDLC frame 
                elif ((self.is_receiving) and
                      (self.rx_byte == self.hdlc.HDLC_FLAG)):
                    logger.debug("run: End of HDLC frame.")

                    # Receive the last byte
                    self.receive_buffer.append(self.rx_byte)
                    
                    # Reset the variables
                    self.is_receiving = False
                    self.last_rx_byte = ''
                    self.rx_byte = ''
                    
                    # Compute statistics
                    self.rx_total_frames += 1
                    
                    try:
                        logger.debug("run: Received an HDLC frame from the Serial port, now de-HDLCifying it.")
                        
                        # Receive me
                        self.receive_message = self.hdlc.dehdlcify(self.receive_buffer)

                        # Compute statistics
                        self.rx_good_frames += 1
                    except:
                        logger.error("run: Error while de-HDLCifying the frame received from the Serial port.")
                        
                        # Clean buffers
                        self.receive_buffer  = []
                        self.receive_message = []

                        # Compute statistics
                        self.rx_bad_frames += 1

                    else:
                         # Acquire the receive condition
                        self.receive_condition.acquire()                   
                        
                         # Notify the receive condition
                        self.receive_condition.notify()
                        
                         # Release the transmit condition
                        self.receive_condition.release()
                        
                        # Reset the receive buffer
                        self.receive_buffer = []
                    
                # Always save the last received byte
                self.last_rx_byte = self.rx_byte

            # If no bytes were received, sleep
            if (rx_length == 0):
                # Sleep for a while
                time.sleep(self.timeout)

        # Close the serial port
        self.serial_port.close()
    
    # Stops the thread
    def stop(self):
        logger.info("stop: Stopping the {} serial port.".format(self.name))

        # Terminates the thread
        self.stop_event.set()
    
    # Receive a message
    def receive(self, timeout = 0.1):
        status  = True
        message = []
        length  = -1
        
        # Acquire the lock
        self.receive_condition.acquire()
        
        # Try to receive a message with timeout
        self.receive_condition.wait(timeout)

        # If we really got a message, copy it!
        if (self.receive_message):
            message = self.receive_message
            length  = len(message)

            logger.info("receive: Received a message with {} bytes.".format(length))
        
        # Reset the receive message
        self.receive_message = []
        
        # Release the receive condition
        self.receive_condition.release()
        
        # Get the status of the thread
        status = self.stop_event.isSet()
        
        # Return the received message and length
        return (message, length)
    
    # Transmit a message
    def transmit(self, message):
        logger.info("transmit: Got a message to transmit with {} bytes.".format(len(message)))
        
        # Compute statistics
        self.tx_total_frames += 1
        
        try:
            logger.debug("transmit: HDLCifying the transmit buffer.")
        
            # HDLCify the message
            self.transmit_buffer = self.hdlc.hdlcify(message)
        except:
            logger.error("transmit: Error HDLCifying the transmit buffer.")
            # Compute statistics
            self.tx_bad_frames += 1
            raise

        try:
            logger.debug("run: Transmitting the message with {} bytes.".format(len(self.transmit_buffer)))
        
            # Send the message through the serial port (blocking)
            self.serial_port.write(self.transmit_buffer)
        except:
            logger.error("transmit: Error transmitting the transmit buffer.")
            # Compute statistics
            self.tx_bad_frames += 1
            raise

        # Compute statistics
        self.tx_good_frames += 1

    def clear_statistics(self):
        self.tx_total_frames = 0
        self.tx_good_frames  = 0
        self.tx_bad_frames   = 0
        self.rx_total_frames = 0
        self.rx_good_frames  = 0
        self.rx_bad_frames   = 0

    def get_statistics(self):
        return "TX Total={}, TX Good={}, TX Bad={}, RX Total={}, RX Good={}, RX Bad={}".format(
            self.tx_total_frames, self.tx_good_frames, self.tx_bad_frames,
            self.rx_total_frames, self.rx_good_frames, self.rx_bad_frames)

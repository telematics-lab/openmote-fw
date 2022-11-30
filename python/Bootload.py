'''
@file       Bootload.py
@author     Pere Tuset-Peiro  (peretuset@openmote.com)
@version    v0.1
@date       May, 2015
@brief

@copyright  Copyright 2015, OpenMote Technologies, S.L.
            This file is licensed under the GNU General Public License v2.
'''

import logging
import os
import subprocess

# Import logging configuration
logger = logging.getLogger(__name__)

class Bootload(object):
    def __init__(self, serial_name = None, serial_speed = 115200, image = None):
        self.python   = "python"
        self.bsl_path = os.path.join("..", "..", "tools", "cc2538-bsl")
        self.bsl_file = "cc2538-bsl.py"

        self.serial_name = serial_name
        self.serial_speed = serial_speed
        self.image = image

        self.bsl_params = "-e -w --bootloader-invert-lines -p {} -b {} {}".format(self.serial_name, self.serial_speed, self.image)

        self.script_cmd = os.path.join(self.bsl_path, self.bsl_file)
        self.script_args = self.bsl_params

    def start(self):
        # Build bootload command
        cmd = self.python + " " + self.script_cmd + " " + self.script_args

        logger.info("start: Executing the bootload command '{}'.".format(cmd))

        try:
            # Execute bootload command
            subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, check=True)

            logger.info("start: Executed the bootload command successfully.")

            return True
        except:
            logger.error("start: Error executing the bootload process.")

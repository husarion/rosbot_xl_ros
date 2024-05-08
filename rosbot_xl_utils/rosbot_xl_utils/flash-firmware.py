#!/usr/bin/python3

# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import sys
import time

import sh
from pyftdi.ftdi import Ftdi

# CBUS0 - BOOT0
# CBUS1 - RST


class FirmwareFlasher:
    def __init__(self, binary_file, port):
        self.device = "ftdi://ftdi:ft-x:/1"
        self.ftdi = Ftdi()

        self.binary_file = binary_file
        self.max_approach_no = 5
        self.port = port

    def enter_bootloader_mode(self):
        self.ftdi.open_from_url(url=self.device)
        self.ftdi.set_cbus_direction(0b11, 0b11)  # set CBUS0 and CBUS1 to output
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b11)  # set CBUS0 to 1 and RST to 1
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b01)  # set CBUS0 to 1 and RST to 0
        time.sleep(0.1)
        # self.ftdi.set_cbus_direction(0b11,0b00) # set CBUS0 and CBUS1 to input
        time.sleep(0.1)
        self.ftdi.close()

    def exit_bootloader_mode(self):
        self.ftdi.open_from_url(url=self.device)
        self.ftdi.set_cbus_direction(0b11, 0b11)  # set CBUS0 and CBUS1 to output
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b10)  # set CBUS0 to 1 and RST to 1
        time.sleep(0.1)
        self.ftdi.set_cbus_gpio(0b00)  # set CBUS0 to 1 and RST to 0
        time.sleep(0.1)
        # self.ftdi.set_cbus_direction(0b11,0b00) # set CBUS0 and CBUS1 to input
        time.sleep(0.1)
        self.ftdi.close()

    def flash_firmware(self):
        self.enter_bootloader_mode()
        sh.usbreset("0403:6015")
        time.sleep(2.0)
        # workaround: using pyftdi causes laggy serial port.
        # This line is like unplug/plug for USB port
        sh.stm32flash(self.port, "-v", w=self.binary_file, b="115200", _out=sys.stdout)
        self.exit_bootloader_mode()
        sh.usbreset("0403:6015")


def main():
    parser = argparse.ArgumentParser(
        description="Flashing the firmware on STM32 microcontroller in ROSbot XL"
    )

    parser.add_argument(
        "-f",
        "--file",
        nargs="?",
        default="/firmware.bin",
        help="Path to a firmware file. Default: /firmware.bin",
    )
    parser.add_argument(
        "-p",
        "--port",
        nargs="?",
        default="/dev/ttyUSB0",
        help="Path to serial connection. Default: /dev/ttyUSB0",
    )

    binary_file = parser.parse_args().file
    port = parser.parse_args().port

    flasher = FirmwareFlasher(binary_file, port)
    flasher.flash_firmware()
    print("Done.")


if __name__ == "__main__":
    main()

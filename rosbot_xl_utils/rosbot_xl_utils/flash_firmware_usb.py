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


class FirmwareFlasherUSB:
    def __init__(self, binary_file, port):
        self.device = "ftdi://ftdi:ft-x:/1"
        self.ftdi = Ftdi()

        self.binary_file = binary_file
        self.max_approach_no = 3
        self.port = port

        self.flash_firmware()

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

    def try_flash_operation(self, operation_name):
        print(f"\n{operation_name} operation started")
        self.enter_bootloader_mode()
        sh.usbreset("0403:6015")
        time.sleep(2.0)
        for i in range(self.max_approach_no):
            print(f"Attempt {i + 1}/{self.max_approach_no}")
            if operation_name == "Flashing":
                flash_args = ["-v", "-w", self.binary_file, "-b", "115200"]
                sh.stm32flash(self.port, *flash_args, _out=sys.stdout)
                print("Success! The robot firmware has been uploaded.")
            elif operation_name == "Write-UnProtection":
                sh.stm32flash(self.port, "-u")
            elif operation_name == "Read-UnProtection":
                sh.stm32flash(self.port, "-k")
            else:
                raise ("Unknown operation.")
            break

        self.exit_bootloader_mode()

    def flash_firmware(self):
        # Disable the flash write-protection
        self.try_flash_operation("Write-UnProtection")

        # Disable the flash read-protection
        self.try_flash_operation("Read-UnProtection")

        # Flashing the firmware
        self.try_flash_operation("Flashing")

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
        default="/dev/ttyUSBDB",
        help="Path to serial connection. Default: /dev/ttyUSBDB",
    )

    binary_file = parser.parse_args().file
    port = parser.parse_args().port

    try:
        FirmwareFlasherUSB(binary_file, port)
    except Exception as e:
        stderr = e.stderr.decode("utf-8")
        if stderr:
            print(f"ERROR: {stderr.strip()}")

if __name__ == "__main__":
    main()

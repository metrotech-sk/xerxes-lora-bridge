#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging

logging.basicConfig(level=logging.INFO)

log = logging.getLogger(__name__)

from colors import red, green, yellow, blue, magenta, cyan, white

from xerxes_protocol import (
    XerxesNetwork,
    DebugSerial,
    XerxesRoot
)

from serial import Serial


class DS(DebugSerial):
    def read(self, size: int = 1) -> bytes:
        """Read data from the serial port.

        Args:
            size (int, optional): Number of bytes to read. Defaults to 1.

        Returns:
            bytes: Data read from the serial port.
        """
        data: bytes = super().read(size)
        try:
            print(red(data.decode("utf-8")), end="")
        except UnicodeDecodeError:
            print(blue(data.hex(":")), end="")
        return data
    
    
class HS(DebugSerial):
    def read(self, size: int = 1) -> bytes:
        data: bytes = super().read(size)
        print(blue(data.hex()), end="")
        return data
        


port = "/dev/ttyACM0"
xn = XerxesNetwork(HS(port))
xn.init(baudrate=921600, timeout=1)

xr = XerxesRoot(0xFE, xn)
xr.ping(0)
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import time

logging.basicConfig(level=logging.INFO)

log = logging.getLogger(__name__)

from colors import red, green, yellow, blue, magenta, cyan, white

from xerxes_protocol import (
    XerxesNetwork,
    DebugSerial,
    XerxesRoot,
    Leaf
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
xn = XerxesNetwork(Serial(port))
xn.init(baudrate=921600, timeout=.1)

xr = XerxesRoot(0xFE, xn)

leaf = Leaf(0x00, xr)

time.sleep(1)

while True:
    try:
        xr.sync()
        time.sleep(.1)
        # print 3 decimals of the mean PV0
        pv0 = f"{25*leaf.pv0:.3f}"
        pv1 = f"{25*leaf.pv1:.3f}"
        pv2 = f"{25*leaf.pv2:.3f}"
        pv3 = f"{25*leaf.pv3:.3f}"
        # print(f"PV0: {green(pv0)}")
        print(f"PV0: {green(pv0)} PV1: {green(pv1)} PV2: {green(pv2)} PV3: {green(pv3)}")   
        time.sleep(.9)
    except TimeoutError:
        log.info("Timeouted")
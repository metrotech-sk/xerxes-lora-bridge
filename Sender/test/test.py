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
        


port = "/dev/ttyACM1"
xn = XerxesNetwork(Serial(port))
xn.init(baudrate=921600, timeout=.1)

xr = XerxesRoot(0xFE, xn)

leaf0 = Leaf(0x00, xr)
leaf1 = Leaf(0x01, xr)

time.sleep(1)

def try_read(leaf: Leaf, exc: Exception = TimeoutError, n_retry: int = 3):
    while(n_retry > 0):
        try:
            return leaf.pv0
        except exc:
            log.debug(f"Exception: {exc}")
            n_retry -= 1
    return None

while True:
    try:
        xr.sync()
        time.sleep(.1)
        # print 3 decimals of the mean PV0
        l0_pv0 = f"{25*try_read(leaf0, TimeoutError):.3f}"
        l1_pv0 = f"{25*try_read(leaf1, TimeoutError):.3f}"
        # print(f"PV0: {green(pv0)}")
        print(f"D0: {green(l0_pv0)} D1: {green(l1_pv0)}")
        time.sleep(.9)
    except TimeoutError:
        log.info("Timeouted")
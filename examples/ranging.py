#!/usr/bin/env python3

from pypozyx import (PozyxSerial, PozyxConstants, DeviceRange, SingleRegister, POZYX_SUCCESS, get_first_pozyx_serial_port)

class Ranger(object):
    def __init__(self, pozyx, dest, remote, protocol=PozyxConstants.RANGE_PROTOCOL_PRECISION):
        self.pozyx = pozyx
        self.dest = dest
        self.remote = remote
        self.protocol = protocol
    
    def setup(self):
        self.pozyx.setRangingProtocol(self.protocol)
    
    def loop(self):
        range = DeviceRange()

        stat = self.pozyx.doRanging(self.dest, range, self.remote)

        if (stat == POZYX_SUCCESS):
            print(range)
        else:
            error_code = SingleRegister()
            status = self.pozyx.getErrorCode(error_code)
            if status == POZYX_SUCCESS:
                print("ERROR Ranging, local %s" %
                      self.pozyx.getErrorMessage(error_code))
            else:
                print("ERROR Ranging, couldn't retrieve local error")


if __name__ == "__main__":
    # the easier way
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    pozyx = PozyxSerial(serial_port)

    dest = 0x671d
    remote = 0x6733

    r = Ranger(pozyx, dest, remote)

    r.setup()

    while True:
        r.loop()

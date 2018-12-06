#!/usr/bin/env python3

from pozyx import (LocalPozyx, RemotePozyx)
import rospy

class PozyxROS(object):

    def __init__(self):
        local = LocalPozyx()

    def setup(self):

    def loop(self):

if __name__ == "__main__":
    pozyxRos = PozyxROS()

    pozyxRos.setup()

    while True:
        pozyxRos.loop()

#!/usr/bin/env python2

from __future__ import division

import rospy
import pypozyx
from pozyx_ros.msg import DeviceRange
from std_msgs.msg import Header

def pozyx_range():
	pub = rospy.Publisher('pozyx_range', DeviceRange, queue_size=100)
	rospy.init_node('range_pub')

	try:
		serial_port = pypozyx.get_first_pozyx_serial_port()

		if serial_port == None:
			rospy.loginfo("Pozyx not connected")
			return

		pozyx = pypozyx.PozyxSerial(serial_port)
	except:
		rospy.loginfo("Could not connect to pozyx")
		return

	# get network id of connected pozyx
	self_id = pypozyx.NetworkID()

	if pozyx.getNetworkId(self_id) != pypozyx.POZYX_SUCCESS:
		rospy.loginfo("Could not get network id")
		return
	
	# discover all other pozyx
	pozyx.doDiscoveryAll()

	devices_size = pypozyx.SingleRegister()

	pozyx.getDeviceListSize(devices_size)

	devices = pypozyx.DeviceList(devices_size)

	pozyx.getDeviceIds(devices)

	for i in range(devices_size[0]):
		rospy.loginfo("Found pozyx device with the id {}".format(pypozyx.NetworkID(devices[i])))        

	# publish data
	while not rospy.is_shutdown():
		for i in range(devices_size[0]):
			device_range = pypozyx.DeviceRange()
			remote_id = pypozyx.NetworkID(devices[i])

			if pozyx.doRanging(self_id[0], device_range, remote_id[0]):
				h = Header()
				h.stamp = rospy.Time.from_sec(device_range[0] / 1000)
				
				pub.publish(h, remote_id[0], device_range[1], device_range[2])
			else:
				rospy.loginfo("Error while ranging")
	

if __name__ == '__main__':
	try:
		pozyx_range()
	except rospy.ROSInterruptException:
		pass

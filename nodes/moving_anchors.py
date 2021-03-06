#!/usr/bin/env python2

from __future__ import division

import rospy
from pozyx_ros import PozyxCluster
from pozyx_ros.msg import DeviceRange
from std_msgs.msg import Header
from geometry_msgs.msg import (PoseStamped, Pose)

class MovingAnchors(object):
	"""
	A pozyx node that accounts for a moving corrdinate system

	Parameters:
	anchors (DeviceCoordinate[]):	the ids and coordinates of the anchors
	tag			(DeviceCoordinate):		the id and coordinate of the tag
	"""
	def __init__(self, anchors=None, tag=None):
		self.range_pub = rospy.Publisher('/pozyx/range', DeviceRange, queue_size=50)
		self.pos_pub = rospy.Publisher('/pozyx/pos', PoseStamped, queue_size=50)

		rospy.init_node('pozyx')

		# TODO: get these from the ros params
		if anchors == None:
			anchors = []

		self.anchors = anchors
		self.tag = tag

		self.cluster = PozyxCluster(anchors, [tag.network_id])

	def start_node(self):
		"""
		Starts the pozxy node
		"""
		while not rospy.is_shutdown():
			try:
				# Get position of tag
				pos = self.cluster.getPosition(self.tag)
				msg = Pose()

				msg.x = pos.x
				msg.y = pos.y
				msg.z = pos.z

				h = Header()
				h.stamp = rospy.Time.now()

				self.pos_pub.publish(h, msg)
			except Exception as e:
				rospy.logerr(e)

			# Get ranges
			for device_id in self.anchors:
				try:
					device_range = self.cluster.rangeTo(device_id, self.tag)

					h = Header()
					h.stamp = rospy.Time.from_sec(device_range[0] / 1000)

					self.range_pub.publish(h, device_id, device_range[1], device_range[2])
				except Exception as e:
					rospy.logerr(e)

if __name__ == "__main__":
	movingAnchors = MovingAnchors()

	try:
		movingAnchors.start_node()
	except rospy.ROSInterruptException:
		pass

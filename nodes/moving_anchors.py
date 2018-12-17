#!/usr/bin/env python2

from __future__ import division

import rospy
from pozyx_ros import PozyxCluster
from pozyx_ros.msg import DeviceRange
from std_msgs.msg import Header
from geometry_msgs.msg import (PoseStamped, Pose)

class MovingAnchors(object):
	def __init__(self, anchors=None, tags=None):
		self.range_pub = rospy.Publisher('/pozyx/range', DeviceRange, queue_size=50)
		self.pos_pub = rospy.Publisher('/pozyx/pos', PoseStamped, queue_size=50)

		if anchors == None:
			anchors = []

		if tags == None:
			tags = []

		self.anchors = anchors
		self.tags = tags

		self.cluster = PozyxCluster(anchors, tags)

	def start(self):
		while not rospy.is_shutdown():
				for id in self.tags:
					try:
						pos = self.cluster.getPosition(id)
						msg = Pose()

						msg.x = pos.x
						msg.y = pos.y
						msg.z = pos.z

						h = Header()
						h.stamp = rospy.Time.now()

						self.pos_pub.publish(h, msg)
					except Exception as e:
						rospy.logerr(e)



				


if __name__ == "__main__":
	movingAnchors = MovingAnchors()

	try:
		movingAnchors.start()
	except rospy.ROSInterruptException:
		pass

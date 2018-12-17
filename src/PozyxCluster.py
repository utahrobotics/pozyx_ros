#!/usr/bin/env python2

import pypozyx
import collections

ERROR_MESSAGES = {
	pypozyx.POZYX_ERROR_NONE: "failed (no error)",
	pypozyx.POZYX_ERROR_I2C_WRITE: "failed on I2C write",
	pypozyx.POZYX_ERROR_I2C_CMDFULL: "failed because I2C command was full",
	pypozyx.POZYX_ERROR_ANCHOR_ADD: "failed to add anchor",
	pypozyx.POZYX_ERROR_COMM_QUEUE_FULL: "failed because command queue is full",
	pypozyx.POZYX_ERROR_I2C_READ: "failed on I2C read",
	pypozyx.POZYX_ERROR_UWB_CONFIG: "failed due to bad UWB config",
	pypozyx.POZYX_ERROR_OPERATION_QUEUE_FULL: "failed because opertion queue is full",
	pypozyx.POZYX_ERROR_TDMA: "failed to do TDMA",
	pypozyx.POZYX_ERROR_STARTUP_BUSFAULT: "failed due to startup bus fault",
	pypozyx.POZYX_ERROR_FLASH_INVALID: "failed due to invalid flash state",
	pypozyx.POZYX_ERROR_NOT_ENOUGH_ANCHORS: "failed due to not enough anchors",
	pypozyx.POZYX_ERROR_DISCOVERY: "failed to do discovery",
	pypozyx.POZYX_ERROR_CALIBRATION: "failed to do calibration",
	pypozyx.POZYX_ERROR_FUNC_PARAM: "failed due to invalid function parameter(s)",
	pypozyx.POZYX_ERROR_ANCHOR_NOT_FOUND: "failed because anchor could not be found",
	pypozyx.POZYX_ERROR_FLASH: "failed to update flash",
	pypozyx.POZYX_ERROR_MEMORY: "failed due to memory fault",
	pypozyx.POZYX_ERROR_RANGING: "failed to do ranging",
	pypozyx.POZYX_ERROR_RTIMEOUT1: "failed because read timedout 1", # not actually sure what this code means
	pypozyx.POZYX_ERROR_RTIMEOUT2: "failed because read timedout 2", # not actually sure what this code means
	pypozyx.POZYX_ERROR_TXLATE: "POZYX_ERROR_TXLATE", # not sure what this means
	pypozyx.POZYX_ERROR_UWB_BUSY: "failed because UWB is busy",
	pypozyx.POZYX_ERROR_POSALG: "POZYX_ERROR_POSALG", # not sure what this means
	pypozyx.POZYX_ERROR_NOACK: "failed because message was not acknowleged",
	pypozyx.POZYX_ERROR_SNIFF_OVERFLOW: "POZYX_ERROR_SNIFF_OVERFLOW", # not sure what this means
	pypozyx.POZYX_ERROR_NO_PPS: "POZYX_ERROR_NO_PPS", # not sure what this means
	pypozyx.POZYX_ERROR_NEW_TASK: "POZYX_ERROR_NEW_TASK", # not sure what this means
	pypozyx.POZYX_ERROR_UNRECDEV: "POZYX_ERROR_UNRECDEV", # not sure what this means
	pypozyx.POZYX_ERROR_GENERAL: "general failure"
}

def getErrorMessage(operationName, code):
	return "{} ({}: {})".format(operationName, code, ERROR_MESSAGES[code])

class PozyxCluster(object):
	"""
	A cluster of pozyx devices

	Parameters:
	anchors (DeviceCorrdinates[]):  the corrdinates of the anchors
	tags (NetworkID[]):             the ids of the tags
	[serial_port] (string):         the serial port for the local pozyx
	"""
	def __init__(self, anchors, tags, serial_port = None):

		if (serial_port is None):
			serial_port = pypozyx.get_first_pozyx_serial_port()

		assert(serial_port is not None, "Could not find local pozyx device")

		self.local = pypozyx.PozyxSerial(serial_port)

		# get local id
		local_id = pypozyx.NetworkID()

		status = self.local.getNetworkId(local_id)
		self._check_status("get network id", status)

		self.local_id = local_id[0]

		# check for nessary devices and set coordinates
		self._find_devices()
		device_list = self._get_device_list()
		device_list.append(self.local_id)

		for device in anchors:
			id = device.network_id
			cord = device.pos

			if id in device_list:
				status = self.local.setCoordinates(cord, None if id == self.local_id else id)
				self._check_status("set coordinates", status)
			else:
				raise Exception("failed to find a pozyx device with the id {:02x}".format(id))

		for id in tags:
			if id not in device_list:
				raise Exception("failed to find a pozyx device with the id {:02x}".format(id))

		# remove any unknown devices
		anchor_ids = map(lambda device: device.network_id, anchors)

		for id in device_list:
			if (id not in anchor_ids) and (id not in tags):
				# TODO: log removal of unknown device
				self.local.removeDevice(id)

		# store lists of anchors and tags
		self.anchors = anchor_ids
		self.tags = tags

	def rangeTo(self, from_id, to_id):
		"""
		Gets the range from one pozyx device in to another

		Paramters:
		from_id (number): the id of the device to measure from
		to_id (number):   the id of the device to measure to

		Returns:
		(DeviceRange):    the range between the devices
		"""
		device_range = pypozyx.DeviceRange()

		status = self.local.doRanging(from_id, device_range, to_id)
		self._check_status("ranging {:02x} -> {:02x}".format(from_id, to_id), status)

		return device_range

	def getPosition(self, tag_id):
		"""
		Gets the position of the given tag

		Parameters:
		tag_id (number):  the id of the tag whose position to get

		Returns:
		(Coordinates):    the position of the tag
		"""

		pos = pypozyx.Coordinates()

		status = self.local.doPositioning(pos, remote_id=tag_id)
		self._check_status("do positioning {:02x}".format(tag_id), status)

		return pos

	def _find_devices(self):
		# discovers all local devices
		status = self.local.doDiscoveryAll()
		self._check_status("discovery", status)

	def _get_device_list(self):
		# gets the device list
		device_list = []
		device_list_size = pypozyx.SingleRegister()

		status = self.local.getDeviceListSize(device_list_size)
		self._check_status("get device list size", status)

		devices = pypozyx.DeviceList(device_list_size.value)

		status = self.local.getDeviceIds(devices)
		self._check_status("get device ids", status)

		for id in devices:
			device_list.append(id)

		return device_list

	def _check_status(self, operation_name, status):
		# checks the status of the operation
		if status == pypozyx.POZYX_SUCCESS:
			return

		if status == pypozyx.POZYX_TIMEOUT:
			raise Exception(operation_name + " timed out")

		error_code = pypozyx.SingleRegister()

		stat = self.local.getErrorCode(error_code)

		if stat != pypozyx.POZYX_SUCCESS:
			raise Exception("failed to get error code")

		code = error_code[0]

		raise Exception(getErrorMessage(operation_name, code))

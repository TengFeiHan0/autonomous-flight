#!/usr/bin/env python

# Simple display window to show result from ORB_SLAM
# and to provide user interaction by overwriting kbd & mouse events

import rospy

from sensor_msgs.msg      import Image
from ardrone_autonomy.msg import Navdata
from threading            import Lock  # To sync GUI and ROS callbacks
from drone_status         import DroneStatus
from PySide               import QtCore, QtGui

class DroneVideoDisplay(QtGui.QMainWindow):
	StatusMessages = {
		DroneStatus.Emergency : 'Emergency',
		DroneStatus.Inited    : 'Initialized',
		DroneStatus.Landed    : 'Landed',
		DroneStatus.Flying    : 'Flying',
		DroneStatus.Hovering  : 'Hovering',
		DroneStatus.Test      : 'Test (?)',
		DroneStatus.TakingOff : 'Taking Off',
		DroneStatus.GotoHover : 'Going to Hover Mode',
		DroneStatus.Landing   : 'Landing',
		DroneStatus.Looping   : 'Looping (?)'
	}

	Disconnected_msg = 'Disconnected'
	Unknown_msg      = 'Unknown Status'

	CONNECTION_CHECK_PERIOD = 250 # ms
	GUI_UPDATE_PERIOD       = 20  # ms
	TAG_RADIUS              = 4

	def __init__(self):
		super(DroneVideoDisplay, self).__init__()

		self.setWindowTitle('ORB_SLAM view')
		self.imageBox = QtGui.QLabel(self)
		self.setCentralWidget(self.imageBox)

		self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.ReceiveNavdata)
		self.subVideo   = rospy.Subscriber('/ORB_SLAM/Frame' , Image  , self.ReceiveImage)
		# self.subVideo   = rospy.Subscriber('/ardrone/image_raw' , Image  , self.ReceiveImage)

		self.image = None
		self.imageLock = Lock()

		self.tags = []
		self.tagLock = Lock()

		self.statusMessage = ''

		# Tracks whether we have received data since the last connection check
		# This works because data comes in at 50Hz but we're checking for a connection at 4Hz
		self.communicationSinceTimer = False
		self.connected = False

		self.connectionTimer = QtCore.QTimer(self)
		self.connectionTimer.timeout.connect(self.ConnectionCallback)
		self.connectionTimer.start(self.CONNECTION_CHECK_PERIOD)

		self.redrawTimer = QtCore.QTimer(self)
		self.redrawTimer.timeout.connect(self.RedrawCallback)
		self.redrawTimer.start(self.GUI_UPDATE_PERIOD)

	# keeps track of traffic
	def ConnectionCallback(self):
		self.connected = self.communicationSinceTimer
		self.communicationSinceTimer = False

	def RedrawCallback(self):
		if self.image is not None:
			self.imageLock.acquire()
			try:
				# Convert the ROS image into a QImage which we can display
				image = QtGui.QPixmap.fromImage(QtGui.QImage(self.image.data, self.image.width, self.image.height, QtGui.QImage.Format_RGB888))

				# Highlight detected tags
				# if len(self.tags) > 0:
				# 	self.tagLock.acquire()
				# 	try:
				# 		painter = QtGui.QPainter()
				# 		painter.begin(image)
				# 		painter.setPen(QtGui.QColor(0,255,0))
				# 		painter.setBrush(QtGui.QColor(0,255,0))
				# 		r = self.TAG_RADIUS
				# 		for (x,y,d) in self.tags:
				# 			rect = QtCore.QRectF((x*image.width())/1000-r,(y*image.height())/1000-r,r*2,r*2)
				# 			painter.drawEllipse(rect)
				# 			painter.drawText((x*image.width())/1000+r,(y*image.height())/1000-r,str(d/100)[0:4]+'m')
				# 		painter.end()
				# 	finally:
				# 		self.tagLock.release()
			finally:
				self.imageLock.release()

			self.resize(image.width(),image.height())
			self.imageBox.setPixmap(image)

		self.statusBar().showMessage(self.statusMessage if self.connected else self.Disconnected_msg)

	def ReceiveImage(self, data):
		self.communicationSinceTimer = True
		self.imageLock.acquire()
		try:     self.image = data          # Save image for display thread
		finally: self.imageLock.release()

	def ReceiveNavdata(self, navdata):
		self.communicationSinceTimer = True
		msg = self.StatusMessages[navdata.state] if navdata.state in self.StatusMessages else self.Unknown_msg
		self.statusMessage = '{} (Battery: {}%)'.format(msg, int(navdata.batteryPercent))

		# Save tags for later processing
		# self.tagLock.acquire()
		# try:
		# 	if navdata.tags_count > 0:
		# 		self.tags = [(navdata.tags_xc[i],navdata.tags_yc[i],navdata.tags_distance[i]) for i in range(0,navdata.tags_count)]
		# 	else:
		# 		self.tags = []
		# finally:
		# 	self.tagLock.release()

if __name__=='__main__':
	import sys
	rospy.init_node('drone_slam_display')
	app = QtGui.QApplication(sys.argv)
	display = DroneVideoDisplay()
	display.show()
	status = app.exec_()
	rospy.signal_shutdown("Terminating")
	sys.exit(status)

#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy 
from std_msgs.msg import Header
import serial
# From Node example class
class KaliPen():
	
	def __init__(self):
		# Create a publisher for joystick button commands
		self.pub=rospy.Publisher('joy',Joy,queue_size=10)
		# Open serial port - hardcoded for now
		self.ser = serial.Serial('/dev/rfcomm0',9600,timeout=0)
		self.joy = Joy()
		self.h = Header()
		self.rate = rospy.Rate(100)
	
	def run(self):
		# Main while loop.
		while not rospy.is_shutdown():
			#read serial port for kalipen msgs
			data = self.ser.readline().decode('ascii').strip()
			if data == "P":
				self.h.stamp = rospy.Time.now()
				self.joy.header = self.h
				self.joy.buttons = [1]
				self.pub.publish(self.joy)
				print(data)
			elif data =="N":
				self.h.stamp = rospy.Time.now()
				self.joy.header = self.h
				self.joy.buttons = [0]
				self.pub.publish(self.joy)
			#print(data)
			self.rate.sleep()
			
if __name__ =='__main__':
# Initialize the node and name it.
	rospy.init_node('KaliPen_joynode')
	# Go to class functions that do all the heavy lifting.
	try:
		ne = KaliPen()
		ne.run()
	except rospy.ROSInterruptException: 
		pass

from time import sleep
import serial
import rospy
from std_msgs.msg import Int32
import struct
ser = serial.Serial('/dev/ttyACM1',115200)
rospy.init_node('odomTalker')
rate = rospy.Rate(100)
a = rospy.Publisher("lwheel",Int32,queue_size=10)
b = rospy.Publisher("rwheel",Int32,queue_size=10)
print("Attempting to communicate with Teensy...")
while (1):
	if ser.in_waiting >= 50:
		ser.read(ser.in_waiting-50)
		x = str(ser.read(50))
		x = x.split("<")[-2]
		x = x.replace(">","")
		x = x.split(",")
		# print(-int(x[0]),int(x[1]))
		a.publish(-int(x[0]))
		b.publish(int(x[1]))
		rate.sleep()
		# sleep(.1) # Delay for one tenth of a second

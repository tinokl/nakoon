#!/usr/bin/env python
import roslib; roslib.load_manifest('nakoon_one')
import smbus
import time
import rospy
bus = smbus.SMBus(1)

from std_msgs.msg import String

address = 0x70

# test string
class sensors:

  def __init__(self):
    rospy.loginfo(rospy.get_name() + ": Starting Sensor Node")
    self.sensors_distance_left_pub = rospy.Publisher('sensors_distance_left', String)

    while not rospy.is_shutdown():
	    write(0x51)
	    #time.sleep(0.7)
	    lightlvl = lightlevel()
	    rng = range()
	    #print lightlvl
	    #print rng
      rospy.loginfo(rospy.get_name() + ": Sending Distance")
      self.sensors_distance_left_pub.publish(String(rng))
      rospy.sleep(1.0)


  def write(value):
    bus.write_byte_data(address, 0, value)
    return -1

  def lightlevel():
    light = bus.read_byte_data(address, 1)
    return light

  def range():
    range_front = bus.read_byte_data(address, 2)
    range_back = bus.read_byte_data(address, 3)
    range_sum = (range_front<<8)+range_back
    return range_sum


if __name__ == '__main__':
  rospy.init_node('sensors')
  try:
    sensors()
  except rospy.ROSInterruptException:
    pass
	

#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from octomap_msgs.msg import ContactSensor, ContactSensorArray
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('test_contact_sensor')
    contact_sensor_array_pub = rospy.Publisher('/contact_sensor_array', ContactSensorArray, latch=True)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        rarm_force_sensor = ContactSensor(header=Header(frame_id="/gazebo_world", stamp=rospy.Time.now()), contact=False, pose=Pose(), meshfile='rarm_force_sensor')
        larm_force_sensor = ContactSensor(header=Header(frame_id="/gazebo_world", stamp=rospy.Time.now()), contact=False, pose=Pose(), meshfile='larm_force_sensor')
        sensor_datas = [rarm_force_sensor, larm_force_sensor]
        contact_sensor_array_pub.publish(sensor_datas)
        r.sleep()


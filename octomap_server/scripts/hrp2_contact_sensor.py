#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from octomap_msgs.msg import ContactSensor, ContactSensorArray
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('test_contact_sensor')
    _frame_id = rospy.get_param('~frame_id', "/base_footprint")
    contact_sensor_array_pub = rospy.Publisher('/contact_sensors_in', ContactSensorArray, latch=True)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        _header = Header(frame_id=_frame_id, stamp=rospy.Time.now())
        msg = ContactSensorArray()
        msg.header = _header
        LARM_LINK0_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK0')
        LARM_LINK1_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK1')
        LARM_LINK2_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK2')
        LARM_LINK3_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK3')
        LARM_LINK4_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK4')
        LARM_LINK5_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK5')
        LARM_LINK6_sensor = ContactSensor(header=_header, contact=False, link_name='LARM_LINK6')
        msg.datas = [LARM_LINK0_sensor, LARM_LINK1_sensor, LARM_LINK2_sensor, LARM_LINK3_sensor, LARM_LINK4_sensor, LARM_LINK5_sensor, LARM_LINK6_sensor]
        contact_sensor_array_pub.publish(msg)
        r.sleep()

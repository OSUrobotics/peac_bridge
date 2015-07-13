#!/usr/bin/env python

from peac_bridge.msg import ControlActuation
from peac_bridge.srv import ReportControlActuation, ReportControlActuationResponse
import rospy
from std_msgs.msg import Time

control_pub = None
heartbeat_pub = None

def publish_heartbeat(evt):
    heartbeat_pub.publish(rospy.Time.now())

def handle_control_actuated(req):
    req.actuation.header.stamp = rospy.Time.now()
    req.actuation.header.frame_id = 'base_link'
    control_pub.publish(req.actuation)
    return ReportControlActuationResponse()

if __name__ == '__main__':
    rospy.init_node('actuation_logging_server')
    control_pub = rospy.Publisher('/control_actuated', ControlActuation)
    rospy.Service('~report_control_actuated', ReportControlActuation, handle_control_actuated)
    heartbeat_pub = rospy.Publisher('/heartbeat', Time)
    timer = rospy.Timer(rospy.Duration(1), publish_heartbeat)
    rospy.spin()

#!/usr/bin/env python

from peac_bridge.msg import ControlActuation
from peac_bridge.srv import ReportControlActuation, ReportControlActuationResponse
import rospy

control_pub = None


def handle_control_actuated(req):
    req.actuation.header.stamp = rospy.Time.now()
    req.actuation.header.frame_id = 'base_link'
    control_pub.publish(req.actuation)
    return ReportControlActuationResponse()

if __name__ == '__main__':
    rospy.init_node('actuation_logging_server')
    control_pub = rospy.Publisher('/control_actuated', ControlActuation)
    rospy.Service('~report_control_actuated', ReportControlActuation, handle_control_actuated)
    rospy.spin()

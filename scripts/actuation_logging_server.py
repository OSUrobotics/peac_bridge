#!/usr/bin/env python

from peac_bridge.msg import ControlActuation
from peac_bridge.srv import ReportControlActuation
import rospy

control_pub = None

def handle_control_actuated(req):
    import pdb; pdb.set_trace()  # breakpoint 3d406f34 //


if __name__ == '__main__':
    rospy.init_node('actuation_logging_server')
    control_pub = rospy.Publisher('/control_actuated', ControlActuation)
    rospy.Service('~report_control_actuated', ReportControlActuation, handle_control_actuated)
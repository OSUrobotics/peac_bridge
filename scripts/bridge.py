#!/usr/bin/env python
import rospy
from peac_bridge.peac_client import PEAC
from peac_bridge.srv import ListDevices, ListLocations, GetDeviceInfo, UpdateControl,\
    ListDevicesResponse, ListLocationsResponse, GetDeviceInfoResponse, UpdateControlResponse,\
    Login, LoginResponse, LoggedInUser, LoggedInUserResponse
from peac_bridge.msg import Location, Device, Control
from threading import Semaphore

class PeacBridge(object):
    def __init__(self, server, user, password):
        self.client = PEAC(server, user, password)
        single_threaded = rospy.get_param('~single_threaded')
        self.sem = Semaphore(1 if single_threaded else 99999)
        rospy.Service('~list_devices', ListDevices, self.handle_list_devices)
        rospy.Service('~list_locations', ListLocations, self.handle_list_locations)
        rospy.Service('~get_device_info', GetDeviceInfo, self.handle_get_device_info)
        rospy.Service('~update_control', UpdateControl, self.handle_update_control)
        rospy.Service('~login', Login, self.handle_login)
        rospy.Service('~current_user', LoggedInUser, self.handle_logged_in_user)

    def handle_list_devices(self, req):
        with self.sem:
            devices = self.client.list_devices(req.locationId)
            return ListDevicesResponse([Device(deviceId=d['id'], name=d['name']) for d in devices])

    def handle_list_locations(self, req):
        with self.sem:
            locations = self.client.list_locations()
            return ListLocationsResponse([Location(locationId=l['id'], name=l['name']) for l in locations])

    def handle_get_device_info(self, req):
        with self.sem:
            controls = self.client.get_device_info(req.deviceId)
            return GetDeviceInfoResponse([Control(controlId=c['id'], numVal=c['numVal'], name=c['name']) for c in controls])

    def handle_update_control(self, req):
        with self.sem:
            control = self.client.update_control(req.controlId, req.numVal)
            return UpdateControlResponse(Control(controlId=control['id'], numVal=control['numVal'], name=control['name']))

    def handle_login(self, req):
        old_client = self.client
        new_client = PEAC(self.client.server, req.user, req.password)
        if new_client.test_credentials():
            self.client = new_client
            return LoginResponse(True)
        return LoginResponse(False)

    def handle_logged_in_user(self, req):
        return LoggedInUserResponse(self.client.user)

if __name__ == '__main__':
    import sys
    SERVER = 'http://localhost:8000'
    USER = ''
    PASS = ''

    argv = rospy.myargv(sys.argv)
    if len(argv) == 4:
        SERVER, USER, PASS = argv[1:]

    rospy.init_node('peac_bridge')
    PeacBridge(SERVER, USER, PASS)
    rospy.spin()
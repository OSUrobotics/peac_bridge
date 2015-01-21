#!/usr/bin/env python
import rospy
from peac_bridge.peac_client import PEAC
from peac_bridge.srv import ListDevices, ListLocations, GetDeviceInfo, UpdateControl,\
    ListDevicesResponse, ListLocationsResponse, GetDeviceInfoResponse, UpdateControlResponse
from peac_bridge.msg import Location, Device, Control

class PeacBridge(object):
    def __init__(self, server, user, password):
        self.client = PEAC(server, user, password)
        rospy.Service('list_devices', ListDevices, self.handle_list_devices)
        rospy.Service('list_locations', ListLocations, self.handle_list_locations)
        rospy.Service('get_device_info', GetDeviceInfo, self.handle_get_device_info)
        rospy.Service('update_control', UpdateControl, self.handle_update_control)

    def handle_list_devices(self, req):
        devices = self.client.list_devices(req.locationId)
        return ListDevicesResponse([Device(deviceId=d['id'], name=d['name']) for d in devices])

    def handle_list_locations(self, req):
        locations = self.client.list_locations()
        return ListLocationsResponse([Location(locationId=l['id'], name=l['name']) for l in locations])

    def handle_get_device_info(self, req):
        controls = self.client.get_device_info(req.deviceId)
        return GetDeviceInfoResponse([Control(controlId=c['id'], numVal=c['numVal'], name=c['name']) for c in controls])

    def handle_update_control(self, req):
        control = self.client.update_control(req.controlId, req.numVal)[0]
        return UpdateControlResponse(Control(controlId=control['id'], numVal=control['numVal'], name=control['name']))

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
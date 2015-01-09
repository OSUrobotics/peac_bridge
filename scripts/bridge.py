#!/usr/bin/env python
import json, requests
import time

SERVER = 'http://localhost:8000'

class PEACInfo:
    def __init__(self, url, method):
        self.url = url
        self.method = method
        self.headers = {'Accept': 'application/json'}

LOCATION_INFO = PEACInfo('/service/locations.json', 'GET')
DEVICES_INFO = PEACInfo('/service/locations/%(locationId)s/devices.json', 'GET')
CONTROLS_INFO = PEACInfo('/service/devics/%(deviceId)s/controls.json', 'GET')
UPDATE_INFO = PEACInfo('/service/controls/update.json', 'PUT')


def _PEAC_request(peacinfo, payload=None, url_args=dict()):
    url = SERVER + peacinfo.url
    return requests.request(peacinfo.method, url % url_args, data=json.dumps(payload), headers=peacinfo.headers)

def list_locations():
    '''
    This requests retrieves all locations.

    Request Type: GET
    Parameters: none
    Response: JSON array with Location Objects.
    '''
    return _PEAC_request(LOCATION_INFO).json()

def list_devices(location_id):
    '''
    This requests gets the list of devices in location location_id

    Request Type: GET
    Parameters: locationId, the id retrieved by the previous call to locations.json
    Response: JSON Array of Device objects.
    '''
    return _PEAC_request(DEVICES_INFO, url_args=dict(locationId=location_id)).json()

def get_device_info(device_id):
    '''
    Retrieves the controls associated with device deviceId.

    Request Type: GET
    Parameters: deviceId, the id retrieved from the device.json call.
    Response: JSON Array of Control objects.
    '''
    return _PEAC_request(CONTROLS_INFO, url_args=dict(deviceId=device_id)).json()

def update_control(id, numval):
    '''
    Updates the control value. This call is used to 'press' a button.

    Method: PUT
    Params: JSON Control Update Object
    Response: Control object
    '''
    return _PEAC_request(UPDATE_INFO, payload=dict(id=id, numval=numval))


def test_server_responses():
    print list_locations()
    print list_devices(83)
    print get_device_info(1955)
    print update_control(5,0)

if __name__ == '__main__':
    test_server_responses()

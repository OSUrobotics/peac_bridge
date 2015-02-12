#!/usr/bin/env python
import json
import requests
from requests.auth import HTTPBasicAuth
import urlparse
import time

class PEACInfo:
    def __init__(self, url, method):
        self.url = url
        self.method = method
        self.headers = {
            'accept': 'application/json',
            'Content-Type': 'application/json'
        }


LOCATION_INFO = PEACInfo('/service/locations.json', 'GET')
DEVICES_INFO = PEACInfo('/service/locations/%(locationId)s/devices.json', 'GET')
CONTROLS_INFO = PEACInfo('/service/devices/%(deviceId)s/controls.json', 'GET')
UPDATE_INFO = PEACInfo('/service/controls/update.json', 'PUT')

class PEAC(object):

    def __init__(self, server, user, password, proxies={}):
        self.server = server
        self.authstr = '%s:%s' % (user,password)
        self.user = user
        self.password = password
        self.proxies = proxies

    def _make_url(self, peacinfo):
        urlparts = list(urlparse.urlparse(self.server + peacinfo.url))
        return urlparse.urlunparse(urlparts)

    def _PEAC_request(self, peacinfo, payload=None, url_args=dict()):
        url = self._make_url(peacinfo)
        if payload:
            resp = requests.request(peacinfo.method, url % url_args, data=json.dumps(payload), headers=peacinfo.headers, auth=HTTPBasicAuth(self.user, self.password), proxies=self.proxies)
        else:
            resp = requests.request(peacinfo.method, url % url_args, headers=peacinfo.headers, auth=HTTPBasicAuth(self.user, self.password), proxies=self.proxies)
        return resp

    def list_locations(self):
        '''
        This requests retrieves all locations.

        Request Type: GET
        Parameters: none
        Response: JSON array with Location Objects.
        '''
        return self._PEAC_request(LOCATION_INFO).json()

    def list_devices(self, location_id):
        '''
        This requests gets the list of devices in location location_id

        Request Type: GET
        Parameters: locationId, the id retrieved by the previous call to locations.json
        Response: JSON Array of Device objects.
        '''
        return self._PEAC_request(DEVICES_INFO, url_args=dict(locationId=location_id)).json()

    def get_device_info(self, device_id):
        '''
        Retrieves the controls associated with device deviceId.

        Request Type: GET
        Parameters: deviceId, the id retrieved from the device.json call.
        Response: JSON Array of Control objects.
        '''
        return self._PEAC_request(CONTROLS_INFO, url_args=dict(deviceId=device_id)).json()

    def update_control(self, controlId, numval):
        '''
        Updates the control value. This call is used to 'press' a button.

        Method: PUT
        Params: JSON Control Update Object
        Response: Control object
        '''
        # import pdb; pdb.set_trace()
        return self._PEAC_request(UPDATE_INFO, payload=dict(id=controlId, numVal=numval)).json()


def test_server_responses():
    import os
    peac = PEAC('http://172.16.20.2', os.environ['PEAC_USER'], os.environ['PEAC_PASSWORD'], proxies={'http': 'http://localhost:8080'})
    print peac.list_locations()
    print peac.list_devices(83)
    print peac.get_device_info(1955)
    print peac.update_control(5,0)

if __name__ == '__main__':
    test_server_responses()

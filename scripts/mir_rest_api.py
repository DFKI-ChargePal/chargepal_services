#!/usr/bin/env python3

import sys
import time
import requests
import hashlib
import base64
import rospy


def mir_rest_api_initialise():

    ## REST API with MiR Robot
    mir_user = rospy.get_param("/mir_user_name")
    mir_password = rospy.get_param("/mir_user_password")
    auth_string = f"{mir_user}:{hashlib.sha256(mir_password.encode()).hexdigest()}"
    base64_auth_string = base64.b64encode(auth_string.encode()).decode('utf-8')
    mir_api_url = "http://mir.com/api/v2.0.0"
    headers = {
        'Content-Type': 'application/json',
        'Accept-Language': 'en_US',
        'Authorization': f'Basic {base64_auth_string}'}

    return headers, mir_api_url
    
def get_map_id(headers,api_url,map):
    map_response = requests.get(api_url+"/maps",headers= headers) 
    if map_response.status_code == 200:
        data = map_response.json()
        for item in data:
            if item['name'] == map:
                map_guid = item['guid']
    
    return map_guid

def get_position_id(headers, api_url, map_id,position):
    poistion_response = requests.get(api_url+"/maps"+map_id+"/positions",headers=headers) 
    if poistion_response.status_code == 200:
        data = poistion_response.json()
        for item in data:
            if item['name'] == position:
                position_guid = item['guid']
    
    return position_guid

def read_position_cb(req):
    
    headers , api_url = mir_rest_api_initialise()
    map_id = get_map_id(headers, api_url, req.map_name)
    position_id = get_position_id(headers, api_url, map_id, req.position_name)
    
    response = requests.get(api_url+"/positions/"+position_id,headers=headers)
    if response.status_code == 200:
        data = response.json()
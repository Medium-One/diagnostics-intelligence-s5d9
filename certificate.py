#! /usr/bin/python3.4

import json
import os
import sys
import subprocess
import random
import string

import requests


URL = 'https://api-rna.mediumone.com'
API_KEY, API_BUSINESS_LOGIN, API_BUSINESS_PASSWD, PROJECT_MQTT_ID, S5D9_PATH = sys.argv[1:6]
if len(sys.argv) > 6:
	BASIC_LOGIN = sys.argv[6]
else:
	BASIC_LOGIN = ''.join(random.choice(string.ascii_uppercase + string.ascii_lowercase + string.digits) for _ in range(12))
if len(sys.argv) > 7:
	BASIC_PASSWD = sys.argv[7]
else:
	BASIC_PASSWD = ''.join(random.choice(string.ascii_uppercase + string.ascii_lowercase + string.digits) for _ in range(12))


REST_WRITE_HEADERS = {'Content-Type': 'application/json', 'Accept': 'application/json'}


def get_cert():
    session = requests.session()

    user_dict = {
        "login_id": API_BUSINESS_LOGIN,
        "password": API_BUSINESS_PASSWD,
        "api_key": API_KEY
    }
    response = session.post('{}/v2/login'.format(URL), data=json.dumps(user_dict),
                            headers=REST_WRITE_HEADERS)

    if response.status_code != 200:
        print("Error logging in", response.content)
        return

    response = session.post("{}/v2/users".format(URL), data=json.dumps({
        "login_id": BASIC_LOGIN,
	"password": BASIC_PASSWD
    }))

    if response.status_code != 200:
        print("Error creating user", response.content)
        return

    user_dict = {
        "login_id": BASIC_LOGIN,
        "password": BASIC_PASSWD,
        "api_key": API_KEY
    }

    response = session.post('{}/v2/login'.format(URL), data=json.dumps(user_dict),
                                headers=REST_WRITE_HEADERS)

    if response.status_code != 200:
        print("Error logging in", response.content)
        return

    response = session.get("{}/v2/users/{}".format(URL, BASIC_LOGIN))

    if response.status_code != 200:
        print("Error retrieving user", response.content)
        return

    basic_user_mqtt_id = response.json()['mqtt_id']

    response = session.post("{}/v2/certs".format(URL), data=json.dumps({
        "api_key": API_KEY
    }))

    if response.status_code != 200:
        print("Error getting cert", response.content)
        return

    body = response.json()
    open('{}/device.crt'.format(S5D9_PATH), 'w').write(body["crt"])
    process = subprocess.Popen(['openssl','rsa','-out', '{}/device.key'.format(S5D9_PATH)],
                             stdin=subprocess.PIPE)
    process.communicate(input=body["key"])
    open('{}/m1config.txt'.format(S5D9_PATH), 'w').write('\n{}'.format(PROJECT_MQTT_ID))
    open('{}/m1user.txt'.format(S5D9_PATH), 'w').write('{}\n'.format(basic_user_mqtt_id))


if __name__ == "__main__":
    get_cert()

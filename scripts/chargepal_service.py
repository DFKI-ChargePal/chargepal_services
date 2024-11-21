#!/usr/bin/env python3
import rospy
import sys
import rospkg
import requests
import hashlib
import base64
import json
import sqlite3
from geometry_msgs.msg import PoseStamped
from datetime import datetime
from typing import Union, Tuple, Dict

rospack = rospkg.RosPack()
sys.path.insert(0, rospack.get_path("chargepal_client") + "/src/chargepal_client")
from chargepal_client.client import Grpc_Client


from chargepal_services.srv import (
    askFreeBCS,
    askFreeBCSResponse,
    askFreeBWS,
    askFreeBWSResponse,
    fetchJob,
    fetchJobResponse,
    updateJobMonitor,
    updateJobMonitorResponse,
    MirReadPositionValue,
    MirReadPositionValueResponse,
    deleteMirMission,
    deleteMirMissionResponse,
    assertLiftValue,
    assertLiftValueResponse,
    askOperationTime,
    askOperationTimeResponse,
    resetStationBlocker,
    resetStationBlockerResponse,
    readRobotCharge,
    readRobotChargeResponse,
    clearMirError,
    clearMirErrorResponse,
    resetIoForCart,
    resetIoForCartResponse,
)

rdb_path = rospy.get_param("/rdb_path")
rdb_copy_path = rospy.get_param("/rdbc_path")
log_file_path = rospy.get_param("/log_file_path")
mir_address = rospy.get_param("/mir_address")
mir_user = rospy.get_param("/mir_user_name")
mir_password = rospy.get_param("/mir_user_password")


def enter_log_file(message: str):
    """
    Writes the given message along with a timestamp to the log file.

    Args:
        message (str): The message to be written to the log file.

    Returns:
        None
    """
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(log_file_path, "a") as log_file:
        log_file.write(f"{timestamp} - {message}\n")


def mir_rest_api_initialise() -> (dict, str):
    """
    Initializes the REST API connection with the MiR Robot.

    Returns:
        A tuple containing the headers (dictionary) and the MiR API URL (string).
    """
    ## REST API with MiR Robot
    username = mir_user
    password = mir_password
    auth_string = f"{username}:{hashlib.sha256(password.encode()).hexdigest()}"
    base64_auth_string = base64.b64encode(auth_string.encode()).decode("utf-8")
    mir_api_url = "http://" + mir_address + "/api/v2.0.0"
    headers = {
        "Content-Type": "application/json",
        "Accept-Language": "en_US",
        "Authorization": f"Basic {base64_auth_string}",
    }

    return headers, mir_api_url


def delete_mir_mission_cb(req) -> deleteMirMissionResponse:
    """
    Deletes the mission queue in the MIR robot.

    Args:
        req: The request object.

    Returns:
        deleteMirMissionResponse: The response object indicating the success of the operation and the status code.

    Raises:
        requests.exceptions.HTTPError: If there is an HTTP error while deleting the mission queue.
    """
    srv_response = deleteMirMissionResponse()
    headers, api_url = mir_rest_api_initialise()

    try:
        api_response = requests.delete(api_url + "/mission_queue", headers=headers)
        api_response.raise_for_status()
        if api_response.status_code == 204:
            srv_response.success = True
        else:
            enter_log_file(
                f"    chargepal_service_ERROR:Unable to delete the mission queue. Status code is {api_response.status_code} "
            )
            srv_response.success = False
    except requests.exceptions.HTTPError as http_err:
        enter_log_file(
            f"    chargepal_service_ERROR:Unable to delete the mission queue. Status code is {api_response.status_code} "
        )
        srv_response.success = False

    srv_response.mir_code = api_response.status_code
    return srv_response


def clear_mir_error_cb(req) -> clearMirErrorResponse:
    """
    Clears the MIR error by sending a PUT request to the MIR REST API.

    Args:
        req: The request object.

    Returns:
        The response object of type clearMirErrorResponse.

    Raises:
        requests.exceptions.HTTPError: If an HTTP error occurs during the request.
    """
    srv_response = clearMirErrorResponse()
    headers, api_url = mir_rest_api_initialise()
    data = {"clear_error": True}

    try:
        api_response = requests.put(api_url + "/status", json=data, headers=headers)
        api_response.raise_for_status()
        if api_response.status_code == 200:
            data = {"state_id": 3}
            api_response = requests.put(api_url + "/status", json=data, headers=headers)
            if api_response.status_code == 200:
                srv_response.success = True
        else:
            enter_log_file(
                f"    chargepal_service_ERROR:Unable to clear mir error. Status code is {api_response.status_code} "
            )
            srv_response.mir_code = api_response.status_code
            srv_response.success = False

    except requests.exceptions.HTTPError as http_err:
        enter_log_file(
            f"    chargepal_service_ERROR:Unable to clear mir error. Status code is {api_response.status_code} "
        )
        srv_response.mir_code = api_response.status_code
        srv_response.success = False

    return srv_response


def get_map_id(headers: str, api_url: str, map: str) -> str:
    """
    Retrieves the map ID for a given map name from the API.

    Args:
        headers (str): The headers to be included in the API request.
        api_url (str): The URL of the API.
        map (str): The name of the map to retrieve the ID for.

    Returns:
        str: The map ID if found, otherwise an empty string.
    """
    map_guid = ""

    try:
        map_response = requests.get(api_url + "/maps", headers=headers)
        map_response.raise_for_status()
        if map_response.status_code == 200:
            data = map_response.json()
            for item in data:
                if item["name"] == map:
                    map_guid = item["guid"]
        else:
            enter_log_file(
                f"    chargepal_service_ERROR:Unable to fetch mapid from mir. Status code is {map_response.status_code} "
            )
            map_response.success = False
    except requests.exceptions.HTTPError as http_err:
        enter_log_file(
            f"    chargepal_service_ERROR:Unable to fetch mapid from mir. Status code is {map_response.status_code} "
        )

    return map_guid


def get_io_id(headers: str, api_url: str) -> str:
    """
    Retrieves the IO modules id (guid) from the MiR API based on the provided headers and API URL.

    Args:
        headers (str): The headers to be used for the API request.
        api_url (str): The URL of the MiR API.

    Returns:
        str: The IO id (guid) retrieved from the MiR API.
    """
    io_guid = ""

    try:
        io_response = requests.get(api_url + "/io_modules", headers=headers)
        io_response.raise_for_status()
        if io_response.status_code == 200:
            data = io_response.json()
            for item in data:
                if item["name"] == "MiR internal IOs":
                    io_guid = item["guid"]
        else:
            enter_log_file(
                f"    chargepal_service_ERROR:Unable to fetch io_guid from mir. Status code is {io_response.status_code} "
            )
            io_response.success = False
    except requests.exceptions.HTTPError as http_err:
        enter_log_file(
            f"    chargepal_service_ERROR:Unable to fetch io_guid from mir. Status code is {io_response.status_code} "
        )

    return io_guid


def get_position_id(headers: str, api_url: str, map_id: str, position: str) -> str:
    """
    Fetches the position ID for a given position name from the MIR API.

    Args:
        headers (str): The headers to be included in the API request.
        api_url (str): The base URL of the MIR API.
        map_id (str): The ID of the map.
        position (str): The name of the position.

    Returns:
        str: The position ID if found, otherwise an empty string.
    """
    position_guid = ""
    try:
        position_response = requests.get(
            api_url + "/maps" + map_id + "/positions", headers=headers
        )
        position_response.raise_for_status()
        if position_response.status_code == 200:
            data = position_response.json()
            for item in data:
                if item["name"] == position:
                    position_guid = item["guid"]

        else:
            enter_log_file(
                f"    chargepal_service_ERROR:Unable to fetch position id from mir. Status code is {position_response.status_code} "
            )
            position_response.success = False
    except requests.exceptions.HTTPError as http_err:
        enter_log_file(
            f"    chargepal_service_ERROR:Unable to fetch position id from mir. Status code is {position_response.status_code}"
        )

    return position_guid


def read_robot_charge_cb(req: readRobotCharge) -> readRobotChargeResponse:
    """
    Callback function to read the charge of the robot.

    Args:
        req (readRobotCharge): The request message containing the robot charge request.

    Returns:
        readRobotChargeResponse: The response message containing the robot charge information.
    """

    service_response = readRobotChargeResponse()

    if not rospy.get_param("/sim_flag"):
        headers, api_url = mir_rest_api_initialise()

        try:
            response = requests.get(api_url + "/status", headers=headers)
            response.raise_for_status()
            if response.status_code == 200:
                data = response.json()
                service_response.robot_charge = float(data["battery_percentage"])
        except requests.exceptions.HTTPError as http_err:
            enter_log_file(
                f"    chargepal_service_ERROR:Unable to fetch robot_status from mir. Status code is {response.status_code} "
            )

    else:
        service_response.robot_charge = 100.00

    return service_response


def read_position_cb(req: MirReadPositionValue) -> MirReadPositionValueResponse:
    """
    Callback function to read the map position from the MIR robot.

    Args:
        req (MirReadPositionValue): The request object containing the map name and position name.

    Returns:
        MirReadPositionValueResponse: The response object containing the target pose.

    Raises:
        requests.exceptions.HTTPError: If there is an error while fetching positions from MIR.
    """

    headers, api_url = mir_rest_api_initialise()
    map_id = get_map_id(headers, api_url, req.map_name)
    position_id = get_position_id(headers, api_url, map_id, req.position_name)

    try:
        response = requests.get(api_url + "/positions/" + position_id, headers=headers)
        response.raise_for_status()
        if response.status_code == 200:
            data = response.json()

            target_pose = PoseStamped()
            target_pose.position.x = data["pos_x"]
            target_pose.position.y = data["pos_y"]
            target_pose.orientation.x = data["orientation"]

            return MirReadPositionValueResponse(target_pose)

    except requests.exceptions.HTTPError as http_err:
        enter_log_file(
            f"    chargepal_service_ERROR:Unable to fetch positions from mir. Status code is {response.status_code} "
        )
        return MirReadPositionValueResponse(None)


def assert_lift_value_cb() -> assertLiftValueResponse:
    """
    Retrieves the lift value from the IO module and returns the corresponding state.

    Returns:
        assertLiftValueResponse: The response object containing the lift state.
    """
    service_response = assertLiftValueResponse()

    if not rospy.get_param("/sim_flag"):
        headers, api_url = mir_rest_api_initialise()
        io_guid = get_io_id(headers, api_url)
        try:
            response = requests.get(
                api_url + "/io_modules/" + io_guid + "/status", headers=headers
            )
            response.raise_for_status()
            if response.status_code == 200:
                data = response.json()
                io_input_state = data["input_state"]
                if io_input_state[2] == True:
                    service_response.state = "down"
                elif io_input_state[3] == True:
                    service_response.state = "up"

        except requests.exceptions.HTTPError as http_err:
            enter_log_file(
                f"    chargepal_service_ERROR:Unable to fetch robot_status from mir. Status code is {response.status_code} "
            )

    else:
        service_response.state = "down"

    return service_response


def fetch_job_cb(req: fetchJob) -> fetchJobResponse:
    """
    Fetches a job using the Grpc_Client instance and returns the job details as a JSON string.

    Args:
        req (fetchJob): The request object containing any required parameters.

    Returns:
        fetchJobResponse: The response object containing the fetched job details as a JSON string.
    """
    client_instance = Grpc_Client()
    response_grpc, status = client_instance.fetch_job()
    service_response = fetchJobResponse()
    if response_grpc != None:
        if response_grpc.job.job_type == "" or response_grpc.job.robot_name == "":
            job = {}
            service_response.job = json.dumps(job, separators=(",", ":"))
        else:
            job = {
                "job_id": response_grpc.job.job_id,
                "job_type": response_grpc.job.job_type,
                "charging_type": response_grpc.job.charging_type,
                "robot_name": response_grpc.job.robot_name,
                "cart": response_grpc.job.cart,
                "source_station": response_grpc.job.source_station,
                "target_station": response_grpc.job.target_station,
            }
            service_response.job = json.dumps(job, separators=(",", ":"))
    service_response.connection_status = status
    return service_response


def ask_free_bcs_cb(req: askFreeBCS) -> askFreeBCSResponse:
    """
    Callback function for asking free BCS (Battery Charging Station) to the server.

    Args:
        req (askFreeBCS): The request message containing the necessary information.

    Returns:
        askFreeBCSResponse: The response message containing the server's response.

    """
    client_instance = Grpc_Client()
    response_grpc, status = client_instance.free_bcs()
    service_response = askFreeBCSResponse()
    if response_grpc != None:
        service_response.station_name = response_grpc.station_name
        if service_response.success:
            enter_log_file(
                f"    Asking free BCS to the server. Suggested BCS is {response_grpc.station_name}"
            )
        else:
            enter_log_file(
                f"    chargepal_service_ERROR: Asking free BCS to the server unsuccessful. Grpc service response is {service_response.success} and connection status is {status}."
            )
    service_response.connection_status = status
    return service_response


def ask_free_bws_cb(req: askFreeBWS) -> askFreeBWSResponse:
    """
    Callback function for asking free BWS.

    Args:
        req (askFreeBWS): The request message containing the necessary information.

    Returns:
        askFreeBWSResponse: The response message containing the server's response.

    """
    client_instance = Grpc_Client()
    response_grpc, status = client_instance.free_bws()
    service_response = askFreeBWSResponse()
    if response_grpc != None:
        service_response.station_name = response_grpc.station_name
        if service_response.success:
            enter_log_file(
                f"    Asking free BWS to the server. Suggested BWS is {response_grpc.station_name}"
            )
        else:
            enter_log_file(
                f"    chargepal_service_ERROR: Asking free BWS to the server unsuccessful. Grpc service response is {service_response.success} and connection status is {status}. "
            )

    service_response.connection_status = status
    return service_response


def update_job_monitor_cb(req: updateJobMonitor) -> updateJobMonitorResponse:
    """
    Callback function for updating the job monitor.

    Args:
        req (updateJobMonitor): The request object containing the job type and job status.

    Returns:
        updateJobMonitorResponse: The response object containing the success status and connection status.
    """
    rospy.loginfo("Updating Job Monitor")
    client_instance = Grpc_Client()
    response_grpc, status = client_instance.update_job_monitor(
        req.job_type, req.job_status
    )
    service_response = updateJobMonitorResponse()
    if response_grpc != None:
        service_response.success = response_grpc.success
        if not service_response.success:
            enter_log_file(
                f"    chargepal_service_ERROR:Updating job monitor unsuccessful. Grpc service response is {service_response.success} and connection status is {status}. "
            )
    service_response.connection_status = status
    return service_response


def ask_operation_time_cb(req: askOperationTime) -> askOperationTimeResponse:
    """
    Callback function for handling the 'ask_operation_time' service request.

    Args:
        req (askOperationTime): The request message containing the cart name.

    Returns:
        askOperationTimeResponse: The response message containing the operation time and connection status.
    """
    client_instance = Grpc_Client()
    response_grpc, status = client_instance.operation_time(req.cart_name)
    service_response = askOperationTimeResponse()
    if response_grpc != None:
        service_response.msec = response_grpc.msec

        enter_log_file(f"    Operating time received as {service_response.msec} .")

    else:
        enter_log_file(
            f"    chargepal_service_ERROR:Fetching operating time unsuccessful. Grpc connection status is {status}. "
        )
    service_response.connection_status = status
    return service_response


def reset_station_blocker_cb(req: resetStationBlocker) -> resetStationBlockerResponse:
    """
    Resets the station blocker based on the request.

    Args:
        req (resetStationBlocker): The request object containing the station information.

    Returns:
        resetStationBlockerResponse: The response object indicating the success and connection status.
    """
    client_instance = Grpc_Client()
    if "BCS" in req.station or "bcs" in req.station:
        response_grpc, status = client_instance.reset_station_blocker(
            "reset_bcs_blocker"
        )

    elif "BWS" in req.station or "bws" in req.station:
        response_grpc, status = client_instance.reset_station_blocker(
            "reset_bws_blocker"
        )

    service_response = resetStationBlockerResponse()
    if response_grpc != None:
        service_response.success = response_grpc.success
    service_response.connection_status = status
    return service_response


def io_module_mir_cb(req: resetIoForCart) -> resetIoForCartResponse:
    """
    Callback function for the IO module in the MIR robot.

    Args:
        req (resetIoForCart): The request message containing the action to perform.

    Returns:
        resetIoForCartResponse: The response message indicating the success of the action.
    """
    service_response = resetIoForCartResponse()

    if not rospy.get_param("/sim_flag"):
        headers, api_url = mir_rest_api_initialise()
        io_guid = get_io_id(headers, api_url)

        if req.action == "pickup_cart":

            try:
                api_response_1 = requests.put(
                    api_url + "/io_modules/" + io_guid + "/status",
                    json={"port": 2, "on": True, "timeout": 5},
                    headers=headers,
                )
                api_response_1.raise_for_status()
                if api_response_1.status_code == 200:
                    api_response_2 = requests.put(
                        api_url + "/io_modules/" + io_guid + "/status",
                        json={"port": 2, "on": False, "timeout": 5},
                        headers=headers,
                    )
                    api_response_2.raise_for_status()

                    if api_response_2.status_code == 200:
                        data = api_response_2.json()
                        io_input_state = data["input_state"]
                        io_output_state = data["output_state"]

                        if io_input_state[2] == True and io_output_state[2] == False:
                            service_response.success = True
                        else:
                            service_response.success = False

            except Exception as e:
                enter_log_file(
                    f"    chargepal_service_ERROR:Unable to set io modules in mir. Error is {e} "
                )

        if req.action == "place_cart":

            try:
                api_response_1 = requests.put(
                    api_url + "/io_modules/" + io_guid + "/status",
                    json={"port": 3, "on": True, "timeout": 5},
                    headers=headers,
                )
                api_response_1.raise_for_status()

                if api_response_1.status_code == 200:

                    api_response_2 = requests.put(
                        api_url + "/io_modules/" + io_guid + "/status",
                        json={"port": 3, "on": False, "timeout": 5},
                        headers=headers,
                    )
                    api_response_2.raise_for_status()

                    if api_response_2.status_code == 200:
                        data = api_response_2.json()
                        io_input_state = data["input_state"]
                        io_output_state = data["output_state"]

                        if io_input_state[3] == True and io_output_state[3] == False:
                            service_response.success = True
                        else:
                            service_response.success = False

            except Exception as e:
                enter_log_file(
                    f"    chargepal_service_ERROR:Unable to set io modules in mir. Error is {e} "
                )

    return service_response


def main():
    rospy.init_node("chargepal_communication_node")
    rospy.Service("/ldb_server/fetch_job", fetchJob, fetch_job_cb)
    rospy.Service("/ldb_server/ask_free_bcs", askFreeBCS, ask_free_bcs_cb)
    rospy.Service("/ldb_server/ask_free_bws", askFreeBWS, ask_free_bws_cb)

    rospy.Service(
        "/ldb_server/update_job_monitor", updateJobMonitor, update_job_monitor_cb
    )
    rospy.Service(
        "/ldb_server/charging_operation_time", askOperationTime, ask_operation_time_cb
    )
    rospy.Service(
        "/ldb_server/reset_station_blocker",
        resetStationBlocker,
        reset_station_blocker_cb,
    )

    rospy.Service("/mir_rest_api/clear_error", clearMirError, clear_mir_error_cb)
    rospy.Service(
        "/mir_rest_api/delete_mission_queue", deleteMirMission, delete_mir_mission_cb
    )
    rospy.Service(
        "/mir_rest_api/assert_lift_value", assertLiftValue, assert_lift_value_cb
    )
    rospy.Service("/mir_rest_api/robot_charge", readRobotCharge, read_robot_charge_cb)
    rospy.spin()


if __name__ == "__main__":
    main()

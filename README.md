The chargepal_services package contains all the ros services servers affliated with the robot. The table below shows the available services and its description.

|Namespace| Name | Description|
|--------| ------ | ----------------- |
|/ldb_server/|  |  |
||fetch_job|Fetch job from the server|
||ask_free_bcs|Ask for a free Battery Charging Station|
||ask_free_bws|Ask for a free Battery Waiting Station|
||update_job_monitor|Update the server regarding the current status of the job|
||charging_operation_time|Ask the server the charging time on the booking|
||reset_station_blocker|Reset a station blocker on the server|
|/mir_rest_api/|||
|| clear_error |Clear an error state on MiR|
||delete_mission_queue|Delete a mission queue on MiR|
|| assert_lift_value |Check the cart assert lift state on MiR|
||robot_charge|Check robot charge on MiR|


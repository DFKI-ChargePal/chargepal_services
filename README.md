The chargepal_services package contains all the ros services servers affliated with the robot. The table below shows the available services and its description.

|Namespace| Name | Description|
|--------| ------ | ----------------- |
|/ldb_server/|  |  |
||fetch_job|Fetch job from the server|
||push_to_ldb|Push contents to the local database (server)|
||ask_free_bcs|Ask for a free Battery Charging Station|
||ask_free_bws|Ask for a free Battery Waiting Station|
||check_ready_to_plugin|Ask the server if plugin process can start at the Adapter Station|
||update_job_monitor|Update the server regarding the current status of the job|
||charging_operation_time|Ask the server the charging time on the booking|
||reset_station_blocker|Reset a station blocker on the server|
|/ldb_server/|  |  |
||update_rdb_copy|Update the Robot Databse Copy|
||verify_rdb_rdb_copy_sync|Check if Robot Databse and Robot Databse Copy are same|
|/mir_rest_api/|||
|| clear_error |Clear an error state on MiR|
||delete_mission_queue|Delete a mission queue on MiR|
|| assert_lift_value |Check the cart assert lift state on MiR|
||robot_charge|Check robot charge on MiR|

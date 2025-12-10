# Como usar commnader_template
Terminal1
. install/setup.bash
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py

Terminal2
. install/setup.bash
ros2 run phantomx_pincher_commander_cpp commander

Terminal3
. install/setup.bash
# para un lado
ros2 topic pub -1 /pose_command phantomx_pincher_interfaces/msg/PoseCommand \
"{x: 0.128, y: 0.0, z: 0.100, roll: 3.142, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
# para abajo 
ros2 topic pub -1 /pose_command phantomx_pincher_interfaces/msg/PoseCommand \
"{x: 0.1, y: 0.0, z: 0.14, roll: 3.142, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
# para home
ros2 topic pub -1 /pose_command phantomx_pincher_interfaces/msg/PoseCommand \
"{x: 0.1, y: 0.0, z: 0.406, roll: 0.0, pitch: 0.0, yaw: 3.142, cartesian_path: false}"

NOTA: Este ultimo permite ver la informació del effector final una vez se ha cambiado la posicion enn moveit
Terminal4
ros2 run tf2_ros tf2_echo phantomx_pincher_base_link phantomx_pincher_end_effector

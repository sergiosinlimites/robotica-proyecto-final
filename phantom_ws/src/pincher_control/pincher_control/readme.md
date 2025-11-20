# Como usar follow_joint_trajectory_node.py

## TERMINAL1
```
ros2 run pincher_control follow_joint_trajectory
```
## TERMINAL2
```
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory   control_msgs/action/FollowJointTrajectory   "{trajectory: {joint_names: [
      phantomx_pincher_arm_shoulder_pan_joint,
      phantomx_pincher_arm_shoulder_lift_joint,
      phantomx_pincher_arm_elbow_flex_joint,
      phantomx_pincher_arm_wrist_flex_joint
    ],
    points: [
      {
        positions: [0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 3, nanosec: 0}
      }
    ]}}"
```

NOTA: HAY QUE REVISAR QUE CUANDO SE APAGUE EL NODO SE APAGUEN LOS MOTORES

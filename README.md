# Use Two Myo to Control PincherX100
* msr final project
* Sonia Yuxiao Lai

## Requirements 
* [interbotix_ros_manipulators](https://github.com/Interbotix/interbotix_ros_manipulators)

## Packacges
* `arm_control`: listens to myo emg and imu info and controls the robot
* `connect_myo`: connects to two myo devices and output result in rviz
    * This package is an ROS adaption from [MioConnect](https://github.com/francocruces/MioConnect)
    * Relative sources also include [myo-raw](https://github.com/dzhu/myo-raw) and [ros_myo](https://github.com/uts-magic-lab/ros_myo)

## Instructions
* `arm_control`
    * `roslaunch arm_control control_robot.launch use_actual:=true` 
    subsribes to myo imu and pose info, then displays the orientation as markers in rviz to simulate arm motion (fixed frame is `myo_raw` for now to visualize both IMU and marker)
    * Use `config/4dof_joint_limits.yaml` to change velocity and acceleration of robot 
* `connect_myo`: 
    * `roslaunch connect_myo myo.launch`
        connects to two myo devices and displays myo  Imu and PoseStamped in rviz 
        * Use `rosparam set /myo_name "['myo_lower', 'myo_upper']"` or `rosparam set /myo_name "['myo_upper', 'myo_lower']"` to assign which myo is on lower/upper arm
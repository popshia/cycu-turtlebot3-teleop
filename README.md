# 10/17 turtlebot3_teleop

## Installations

1. Dependencies
```
$ sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy \
  ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc \
  ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
  ros-melodic-rosserial-arduino ros-melodic-rosserial-python \
  ros-melodic-rosserial-server ros-melodic-rosserial-client \
  ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
  ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro \
  ros-melodic-compressed-image-transport ros-melodic-rqt* \
  ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```
2. TurtleBot3 packages
```
$ sudo apt-get install ros-melodic-dynamixel-sdk
$ sudo apt-get install ros-melodic-turtlebot3-msgs
$ sudo apt-get install ros-melodic-turtlebot3
```
## Configs

1. Set TurtleBot3 model name
```
$ echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```
2. IP settings ( check your vm ip with `ifconfig` )
```
$ nano ~/.bashrc
```
> Press `Alt`+`/` to move the cursor to the end of the file

> Modify the address of `MASTER_URL` to
```
export ROS_MASTER_URL=http://${IP_ADDRESS_OF_YOUR_VM}:11311
```
> Modify the address of `HOSTNAME`
```
export ROS_HOSTNAME=${IP_ADDRESS_OF_YOUR_VM}
```
> Exit nano and source the file with
```
$ source ~/.bashrc
```
## TurtleBot3

1. SSH to your TurtleBot3
```
ssh pi@{IP_ADDRESS_OF_TURTLEBOT3}
```
2. IP settings ( Check TurtleBot3 ip with `ifconfig` )
```
$ nano ~/.bashrc
```
> Press `Alt`+`/` to move the cursor to the end of the file

> Modify the address of `MASTER_URL` to
```
export ROS_MASTER_URL=http://${IP_ADDRESS_OF_YOUR_VM}:11311
```
> Modify the address of `HOSTNAME`
```
export ROS_HOSTNAME=${IP_ADDRESS_OF_TURTLEBOT3}
```
> Exit nano and source the file with
```
$ source ~/.bashrc
```
3. Run TurtleBot basic packages
```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
> The terminal will print the SUMMARY messages
```
 SUMMARY
 ========

 PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.3
 * /turtlebot3_core/baud: 115200
 * /turtlebot3_core/port: /dev/ttyACM0
 * /turtlebot3_core/tf_prefix:
 * /turtlebot3_lds/frame_id: base_scan
 * /turtlebot3_lds/port: /dev/ttyUSB0

 NODES
 /
     turtlebot3_core (rosserial_python/serial_node.py)
     turtlebot3_diagnostics (turtlebot3_bringup/turtlebot3_diagnostics)
     turtlebot3_lds (hls_lfcd_lds_driver/hlds_laser_publisher)

 ROS_MASTER_URI=http://192.168.1.2:11311

 process[turtlebot3_core-1]: started with pid [14198]
 process[turtlebot3_lds-2]: started with pid [14199]
 process[turtlebot3_diagnostics-3]: started with pid [14200]
 [INFO] [1531306690.947198]: ROS Serial Python Node
 [INFO] [1531306691.000143]: Connecting to /dev/ttyACM0 at 115200 baud
 [INFO] [1531306693.522019]: Note: publish buffer size is 1024 bytes
 [INFO] [1531306693.525615]: Setup publisher on sensor_state [turtlebot3_msgs/SensorState]
 [INFO] [1531306693.544159]: Setup publisher on version_info [turtlebot3_msgs/VersionInfo]
 [INFO] [1531306693.620722]: Setup publisher on imu [sensor_msgs/Imu]
 [INFO] [1531306693.642319]: Setup publisher on cmd_vel_rc100 [geometry_msgs/Twist]
 [INFO] [1531306693.687786]: Setup publisher on odom [nav_msgs/Odometry]
 [INFO] [1531306693.706260]: Setup publisher on joint_states [sensor_msgs/JointState]
 [INFO] [1531306693.722754]: Setup publisher on battery_state [sensor_msgs/BatteryState]
 [INFO] [1531306693.759059]: Setup publisher on magnetic_field [sensor_msgs/MagneticField]
 [INFO] [1531306695.979057]: Setup publisher on /tf [tf/tfMessage]
 [INFO] [1531306696.007135]: Note: subscribe buffer size is 1024 bytes
 [INFO] [1531306696.009083]: Setup subscriber on cmd_vel [geometry_msgs/Twist]
 [INFO] [1531306696.040047]: Setup subscriber on sound [turtlebot3_msgs/Sound]
 [INFO] [1531306696.069571]: Setup subscriber on motor_power [std_msgs/Bool]
 [INFO] [1531306696.096364]: Setup subscriber on reset [std_msgs/Empty]
 [INFO] [1531306696.390979]: Setup TF on Odometry [odom]
 [INFO] [1531306696.394314]: Setup TF on IMU [imu_link]
 [INFO] [1531306696.397498]: Setup TF on MagneticField [mag_link]
 [INFO] [1531306696.400537]: Setup TF on JointState [base_link]
 [INFO] [1531306696.407813]: --------------------------
 [INFO] [1531306696.411412]: Connected to OpenCR board!
 [INFO] [1531306696.415140]: This core(v1.2.1) is compatible with TB3 Burger
 [INFO] [1531306696.418398]: --------------------------
 [INFO] [1531306696.421749]: Start Calibration of Gyro
 [INFO] [1531306698.953226]: Calibration End
```
## turtlebot3_teleop_key

1. Run `roscore` in your vm.
```
$ roscore
```
2. Launch `turtlebot3_teleop_key`
```
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
3. If the node is successfully launched, the following instruction will be appeared to the terminal window.
```
Control Your Turtlebot3
Moving around
     w
 a   s   d
     x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit
```
## Topic Script

1. In your vm, switch the directory to `catkin_ws/src`
```
$ cd ~/catkin_ws/src
```
2. Clone the repository.
```
$ git clone https://github.com/popshia/turtlebot3_teleop_new
```
3. `cd` into the python file directory.
```
$ cd turtlebot3_teleop_new/nodes
```
4. Look into the python file and call each function in main with your parameters.
```
$ nano turtlebot3_teleop_key
```
5. After saving the script file, `catkin_make` the script.
```
$ cd ~/catkin_ws
$ catkin_make --only-pkg-with-deps turtlebot3_teleop_new
```
## Run the file

1. `rosrun` the package we just make using `catkin make`.
```
$ rosrun turtlebot3_teleop_new turtlebot3_teleop_key
```

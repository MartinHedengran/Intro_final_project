# Intro_final_project
The following git is for solutions for week 19, 20, 21 and week 22 of intro to dronetechnology course at SDU 2020

Commands for the terminal:
```shell
source /home/$USER/Firmware/Tools/setup_gazebo.bash /home/$USER/Firmware /home/$USER/Firmware/build/px4_sitl_default

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Firmware

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/$USER/Firmware/Tools/sitl_gazebo
```

To launch mavros run this code in terminal:
```shell
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
To launch week 19 solution you need to download ros workspace and rosbag file which is avaiable on Blackborad. After that, download all files which is located in the path below and put in ros worksapce:
```shell
Firmware/Tools/sitl_gazebo/src/mission_path/src
```
Open new terminal in ros workspace and run this command:
```shell
source devel/setup.bash
rosbag play manual_flight_rosbag.bag
rosrun mavlink_lora gcs_rosbag_data.py
python outlier_simplification.py
```
When you got waypoints and need to convert as format mission for QGroundControl and Upload it. Now exceute mission and in ros worksapce run this command:

```shell
rosrun mavlink_lora gcs_mission_recored.py
```
After we recored new data we convert as format kml and run this command:
```shell
python Make_kml.py
```


To launch week 20 solution run these codes in different terminals:
```shell
roslaunch px4 posix_sitl.launch vehicle:=iris_fpv_cam world:=/home/gazebo/Firmware/Tools/sitl_gazebo/worlds/aruco.world

python Firmware/Tools/sitl_gazebo/src/offboard_control/src/python_control.py
```


To launch week 21 solution run these codes in different terminals:
```shell
roslaunch px4 posix_sitl.launch vehicle:=iris_fpv_cam_parachute world:=/home/gazebo/Firmware/Tools/sitl_gazebo/worlds/aruco.world

python Firmware/Tools/sitl_gazebo/src/offboard_control/src/python_controlweek21.py
```

To launch week 22 wind simulation run this in the terminal:
```shell
roslaunch px4 posix_sitl.launch vehicle:=iris_fpv_cam_wind world:=/home/gazebo/Firmware/Tools/sitl_gazebo/worlds/aruco.world
```




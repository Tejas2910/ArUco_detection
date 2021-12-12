## Proceeding to detect ArUco..

Open the Terminal and run following commands-
```
cd ~/catkin_ws/src
git clone https://github.com/Tejas2910/aruco_detection/tree/python3_noetic
cd ~/catkin_ws
catkin_make
```
Now you have a package aruco_detection, let's run it.
```
roslaunch aruco_detection maze_aruco.launch
```
Let's spwan the Turtlebot3 by running follwing command in another tab
```
roslaunch aruco_detection spawn_turtlebot3.launch
```
You can see ArUco marker in front of TurtleBot3(waffle_pi model).
Why we used waffle_pi ? Guess... Remember Investigation 1 of Episode 1. 

Yes, you guessed correctly. Let's check by executing ``` rostopic list ``` in another tab.
```
/clock
/cmd_vel
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/imu
/joint_states
/odom
/rosout
/rosout_agg
/scan
/tf
/turtlebot3_waffle_pi/camera/camera_info
/turtlebot3_waffle_pi/camera/image_raw
/turtlebot3_waffle_pi/camera/image_raw/compressed
/turtlebot3_waffle_pi/camera/image_raw/compressed/parameter_descriptions
/turtlebot3_waffle_pi/camera/image_raw/compressed/parameter_updates
/turtlebot3_waffle_pi/camera/image_raw/compressedDepth
/turtlebot3_waffle_pi/camera/image_raw/compressedDepth/parameter_descriptions
/turtlebot3_waffle_pi/camera/image_raw/compressedDepth/parameter_updates
/turtlebot3_waffle_pi/camera/image_raw/theora
/turtlebot3_waffle_pi/camera/image_raw/theora/parameter_descriptions
/turtlebot3_waffle_pi/camera/image_raw/theora/parameter_updates
/turtlebot3_waffle_pi/camera/parameter_descriptions
/turtlebot3_waffle_pi/camera/parameter_updates
```
Camera Sensor is publishing data of ```sensor_msgs/Image``` msg type to ```/turtlebot3_waffle_pi/camera/image_raw``` topic. Let's visulise this data throgh 


## Let's Solve MaZe
At this stage, you have enough knowledge to escape from the maze created by Moriarty.

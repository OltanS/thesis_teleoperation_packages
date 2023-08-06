# Undergraduate Thesis for Oltan Sevinc

This repository includes files that relate to teleoperating a Universal Robots E-series robot via a 3Dsystems Touch haptic device with haptic feedback for the purposes of improving remote ultrasounds.

The steps below are not written in great detail and assume familiarity with ROS and enough knowledge of ubuntu to bugsplat as issues come up, but should be sufficient to point the users in the correct direction.

Steps to utilize the code:
1. Get an installation of Ubuntu 20.04
2. (Optional but highly recommended) Install a realtime kernel on it
3. Setup ROS Noetic on your installation
4. Install catkin_tools
5. Create a catkin workspace
6. Install the touch driver from [here](https://github.com/bharatm11/Geomagic_Touch_ROS_Drivers/tree/hydro-devel) in your catkin workspace and follow the instructions it presents.
7. Install the universal robots ROS driver from [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) in your catkin workspace and follow its setup instructions up to the end of extracting calibration information.
8. Install moveit_servo following the instructions [here](https://ros-planning.github.io/moveit_tutorials/doc/realtime_servo/realtime_servo_tutorial.html) in your catkin workspace.
9. Download this repository into your catkin workspace.
10. Build and source your catkin workspace. (use catkin build not catkin_make)
11. Run one of the launch files to start, like ```roslaunch ur_to_touch_haptic_teleoperation ur_touch_haptic_teleoper.launch```

# Home-Service-Robot
### Project Purpose
This project simulates an autonomous robot that picks up an object and drop it off at predetermined locations. 

### Packages Used:
#### Ros Packages
* [slam_gmapping](https://github.com/ros-perception/slam_gmapping)
  * This package creates a map of the environment using laser-based SLAM.
* [turtlebot](https://github.com/turtlebot/turtlebot)
  * Used turtlebot_teleop  to manually drive the robot from the keyboard in some the scripts file.
* [turtlebot_interactions](https://github.com/turtlebot/turtlebot_interactions)
   * This package is used to launch RViz which allows the user to visualize SLAM, particle filters and send nav goals. Here RViz config files can be saved which will launch the map, robot model, markers etc.
* [turtlebot_simulator](https://github.com/turtlebot/turtlebot_simulator)
  * This package contains the turtlebot_gazebo file which allows you to launch the turtlebot world  with the turtlebot(turtlebot_world.launch) and launches the amcl algorithm (amcl_demo.launch). The robot is able to localize itself using the amcl package. 

#### Packages created by me
* [pick_objects](https://github.com/prasunnyD/Home-Service-Robot/tree/master/src/pick_objects)
  * This package sends pick up and drop off goals to the robot which then uses the ROS navigation stack to reach the goal. Pick_objects package drives the navigation of the robot.
* [add_markers](https://github.com/prasunnyD/Home-Service-Robot/tree/master/src/add_markers)
  * This package publishes a virtual object in Rviz that appears in the pick up goal location. Once the robot reaches the pick up location the object disappears and the robot is then directed to the drop off goal location by the pick_objects package. Once the robot reaches the drop off the goal location then the object appears. 

To launch the home service, type in these commands in the catkin workspace. 
```
> catkin_make
> source devel/setup.bash
> cd src/scripts
> ./home_service.sh
```

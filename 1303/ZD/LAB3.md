# Package installation
In your workspace/src directory run:
```sh
$ ln -s [path-to-cloned-repo]/1303/ZD/3/lost_robot lost_robot
$ ln -s [path-to-cloned-repo]/1303/ZD/3/helper_robot helper_robot
```
# Execution
Run rviz with config file: ROS-course/1303/ZD/3/rviz_config.rviz
Run in separate terminals:
```sh
$ rosrun lost_robot lost_robot
$ rosrun helper_robot helper_robot
```

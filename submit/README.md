# COMP 603 Project 3 D1

**Author:** Li Chen (lchen0@umass.edu)

Please move `move_model` and `sensor_model` directories into `src/` of workspace. The workspace should also contains `turtlebot3_simulation` package.

Under `/catkin_ws` directory, run
```
    $ catkin_make 
```

To launch the `motion_model` and `sensor_model`, you should open multiple terminal. Each terminal should source `noetic/setup.bash` and `catkin_ws/devel/setup.bash`

You should:
1. Run `roscore`
2. Run `roslaunch sensor_model sensor.launch`. This should turn on gazabo simulation and rviz interface.
3. Run `rosrun motion_model motion.py`
4. Run `roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch `
5. Under `sensor_model/src/`, run `python sensor.py`

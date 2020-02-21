# CS8803 (Machine Learning for Robotics) Projects
![](../media/p1.gif)
## Dependencies
- ROS
- V-REP
- tmuxinator (optional)
- Caffe
## Config
- Add V-REP commands/aliases to shell   
```
alias vrep="sh path/to/vrep.sh"
export VREP_SCENES="path/to/vrep/scenes"
```

- Generate compilation database for clang tools (like vim YouCompleteMe)
```
catkin-make -DCMAKE_EXPORT_COMPILE_COMMANDS=1
ln -s ROS_WS/build/compile_commands.json
```
## Tested On
- Ubuntu 18.04
- ROS Kinetic

## Authors
- Ayush Shukla
- Bilal Ghader

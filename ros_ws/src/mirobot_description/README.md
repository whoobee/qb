# mirobot_description package

### xacro

To convert the xacro file into a URDF file:
```bash
$ roscd mirobot_description/urdf/
$ rosrun xacro xacro --inorder mirobot.xacro > mirobot.urdf
```

### URDF

To check whether the sintax is fine or whether it have errors:
```bash
$ check_urdf mirobot.urdf
```

To get the Graphviz in pdf:
```bash
$ sudo apt-get install graphviz
$ urdf_to_graphviz mirobot.urdf
```

### Test

To run rviz test:
```bash
$ roslaunch mirobot_description mirobot_rviz.launch
```

![Chassis](../resources/robot_chassis.jpg)

![Velodyne](../resources/velodyne_drawing.png)

![GraphViz](../resources/graphviz.png)

# qb_description package

### xacro

To convert the xacro file into a URDF file:
```bash
$ roscd qb_description/urdf/
$ rosrun xacro xacro --inorder qb.xacro > qb.urdf
```

### URDF

To check whether the sintax is fine or whether it have errors:
```bash
$ check_urdf qb.urdf
```

To get the Graphviz in pdf:
```bash
$ sudo apt-get install graphviz
$ urdf_to_graphviz qb.urdf
```

### Test

To run rviz test:
```bash
$ roslaunch qb_description qb_rviz.launch
```

![Chassis](../resources/robot_chassis.jpg)

![Velodyne](../resources/velodyne_drawing.png)

![GraphViz](../resources/graphviz.png)

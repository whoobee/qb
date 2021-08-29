#!/usr/bin/env bash

echo "Cleaning sdf and urdf files"
rm -fr qb.urdf qb.sdf
rosrun xacro xacro -o qb.urdf ../urdf/qb.xacro
gz sdf -p qb.urdf > qb.sdf
echo "Generated SDF file"
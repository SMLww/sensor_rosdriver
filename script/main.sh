#!/bin/bash

gnome-terminal --title="MPMGS201 Node" -- bash -c "
cd ~/Documents/sensor_rosdriver; 
cd devel;
source setup.bash;
cd ..;
roslaunch memsplus_sensor_ros mpmgs201.launch; 
exec bash
"

gnome-terminal --title="MPRID1356 Node" -- bash -c "
cd ~/Documents/sensor_rosdriver; 
cd devel;
source setup.bash;
cd ..;
roslaunch memsplus_sensor_ros mprid1356.launch; 
exec bash
"

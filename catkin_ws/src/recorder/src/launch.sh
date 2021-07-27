#!/bin/bash

path="/root/SSD"
#date=`date "+%m%d_%H%M"`
date=`date "+%H-%M-%S-%m%d"`

mkdir $path/$date

#roslaunch recorder recorder.launch path:=$path/$date/
roslaunch signal_process tdoa.launch path:=$path/$date/





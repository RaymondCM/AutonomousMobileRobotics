#!/bin/bash 

git config user.email "ray.tunstill@live.co.uk"
git config user.name "RaymondKirk"
catkin_make
source ./devel/setup.bash
spyder

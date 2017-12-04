#!/bin/bash

clear

if [ -f "devel/setup.sh" ]; then
   source devel/setup.bash
else
   source /opt/ros/indigo/setup.sh
fi

BUILD_TYPE=""
CALIBRATION_MODE=0

for i in "$@"
do
case $i in
    -cm)
    CALIBRATION_MODE=1
    shift # past argument=value
    ;;
    -s)
    BUILD_TYPE="-s"
    shift # past argument=value
    ;;
    *)
    # unknown command
    ;;
esac
done

bash compile.bash ${BUILD_TYPE}

if [ $CALIBRATION_MODE -eq 1 ]; then
    roslaunch cic camera_adjustment.launch
else
    roslaunch cic full.launch
fi

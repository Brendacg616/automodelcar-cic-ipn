clear

if [ -f "devel/setup.sh" ]; then
   source devel/setup.bash
else
   source /opt/ros/indigo/setup.sh
fi

BAG="$@"

rosbag play bags/$BAG.bag -l
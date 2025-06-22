
if [ "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp" ];
then
    . /home/ros/uwrt-config/source/fastdds_server.bash
elif [ "$RMW_IMPLEMENTATION" = "rmw_zenoh_cpp" ];
then
    . /home/ros/uwrt-config/source/zenoh_server.bash
fi

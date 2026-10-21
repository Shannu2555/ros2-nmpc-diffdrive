#!/bin/bash
# One full run inside the container: Gazebo (headless) + RViz on a virtual
# display + controller. Usage: run_in_docker.sh <mpc|pure_pursuit> <figure_eight|sharp_corners> <out_dir>
set -e
CTRL=$1; PATH_NAME=$2; OUT=$3
source /opt/ros/humble/setup.bash
mkdir -p /ws/src && ln -sfn /repo /ws/src/mpc_diffdrive
cd /ws && colcon build --symlink-install >/dev/null 2>&1 || colcon build --symlink-install
source /ws/install/setup.bash
export ROS_DOMAIN_ID=$((RANDOM % 100))
Xvfb :99 -screen 0 1280x720x24 >/dev/null 2>&1 &
export DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1
sleep 2
ros2 launch mpc_diffdrive sim.launch.py gz_extra:=-s rviz:=true > $OUT/sim_${CTRL}_${PATH_NAME}.log 2>&1 &
SIM=$!
sleep 15
ffmpeg -y -loglevel error -f x11grab -video_size 1280x720 -framerate 15 -i :99 \
  -c:v libx264 -pix_fmt yuv420p $OUT/rviz_${CTRL}_${PATH_NAME}.mp4 &
FF=$!
ros2 launch mpc_diffdrive track.launch.py controller:=$CTRL path:=$PATH_NAME out_dir:=$OUT \
  > $OUT/track_${CTRL}_${PATH_NAME}.log 2>&1 || true
sleep 2
kill -INT $FF; wait $FF || true
kill -INT $SIM; sleep 3; pkill -f gz || true; pkill -f rviz2 || true

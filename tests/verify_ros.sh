#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
export COURSE_RESULTS_DIR="$ROOT/../results"
: "${ROBOT_REPLAY_VIDEO:?Set ROBOT_REPLAY_VIDEO to the original video path}"
mkdir -p "$ROOT/../results"
env -i PATH=/usr/bin:/bin HOME=/home/hp bash --noprofile --norc -c 'source /opt/ros/noetic/setup.bash; rospack find sfd_jixiebi' > "$ROOT/../results/error_package.txt" 2>&1 || true
env -i PATH=/usr/bin:/bin HOME=/home/hp bash --noprofile --norc -c 'source /opt/ros/noetic/setup.bash; python3 -c "from sfd_jixiebi.demo_scene import COLOR_ORDER"' > "$ROOT/../results/error_python_path.txt" 2>&1 || true
source /opt/ros/noetic/setup.bash
cd "${ROBOT_WS:-$HOME/robotlearning_ws}"
catkin_make --pkg sfd_jixiebi > "$ROOT/../results/catkin_build.txt" 2>&1
source devel/setup.bash
rospack find sfd_jixiebi > "$ROOT/../results/fixed_package.txt"
python3 -c 'from sfd_jixiebi.demo_scene import COLOR_ORDER; print("Python package OK",COLOR_ORDER)' > "$ROOT/../results/fixed_python_path.txt"
check_urdf src/sfd_jixiebi/urdf/sfd_arm.urdf > "$ROOT/../results/check_urdf.txt"
export ROS_MASTER_URI=http://localhost:11341
export ROS_IP=127.0.0.1
export ROS_LOG_DIR="$ROOT/ros_logs"
export PYTHONUNBUFFERED=1
roslaunch -p 11341 sfd_jixiebi grasp_demo.launch rviz:=false loop:=false speed:=4 > "$ROOT/../results/roslaunch.txt" 2>&1 &
launch_pid=$!
trap 'kill -INT "$launch_pid" 2>/dev/null || true; wait "$launch_pid" 2>/dev/null || true' EXIT
for i in $(seq 1 20); do if rostopic list > /dev/null 2>&1; then break; fi; sleep .5; done
python3 "$ROOT/ros_probe.py" > "$ROOT/../results/probe_stdout.txt" 2>&1 &
probe_pid=$!
rostopic echo -n 1 /joint_states > "$ROOT/../results/joint_echo.txt"
rostopic echo -n 1 /camera/image_raw/encoding > "$ROOT/../results/image_encoding.txt"
timeout --signal=INT 6s rostopic hz /joint_states > "$ROOT/../results/joint_hz.txt" 2>&1 || true
timeout --signal=INT 4s rostopic hz /camera/image_raw > "$ROOT/../results/image_hz.txt" 2>&1 || true
wait "$probe_pid"
rostopic list > "$ROOT/../results/topics.txt"

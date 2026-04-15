colcon build --packages-select jetracer --cmake-clean-cache
source install/setup.bash
ros2 launch jetracer jetracer.launch.py \
  use_sim_time:=false agent_name:=agent_0
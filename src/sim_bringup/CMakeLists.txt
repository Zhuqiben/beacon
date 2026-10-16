cmake_minimum_required(VERSION 3.8)
project(sim_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

# Command line tools, so `ros2 run sim_bringup <tool>` works.
# world_setup.py and world_rules.py must be installed together: the launch file runs
# them as subprocesses by absolute path, and if either is missing the world silently
# falls back to the fixed template instead of a randomised one.
install(PROGRAMS
  scripts/benchmark.py
  scripts/visibility_map.py
  scripts/tf_relay.py
  scripts/dynamic_obstacles.py
  scripts/world_setup.py
  scripts/world_rules.py
  DESTINATION lib/${PROJECT_NAME}
  OPTIONAL
)

ament_package()

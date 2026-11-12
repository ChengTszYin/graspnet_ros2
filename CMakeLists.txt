cmake_minimum_required(VERSION 3.8)
project(graspnet_ros2)

# ament_cmake (not ament_python) is required so the custom DetectGrasps service
# is generated inside this package, keeping the type graspnet_ros2/srv/DetectGrasps.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/DetectGrasps.srv"
  "srv/DetectGraspsMask.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)

# rosidl already installs the generated graspnet_ros2 Python package (srv/).
# Install the node module into that same namespace directly; calling
# ament_python_install_package again would duplicate its CMake target.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(DIRECTORY graspnet_ros2/
  DESTINATION "${PYTHON_INSTALL_DIR}/graspnet_ros2"
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  graspnet_ros2/graspnet_node.py
  script/draw_bbox_grasp.py
  script/sam_grasp.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_dependencies(rosidl_default_runtime)
ament_package()
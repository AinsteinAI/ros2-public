# 1. Minimum required CMake version (ROS 2 Humble requires ≥3.20)
cmake_minimum_required(VERSION 3.20)

# 2. Package name (must match <name> in package.xml)
project(radar_msgs)

# 3. Compilation options (enable warnings, improve code quality)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 4. Find required packages (basic packages for message compilation)
# - ament_cmake: ROS 2 core build tools
# - std_msgs: dependency on standard messages (e.g., Header type)
# - rosidl_default_generators: message code generation tool
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 5. Declare .msg message file list (paths must match actual files)
# Order: basic messages (Detection/Object) first, list messages (DetectionList/ObjectList) last
set(msg_files
  "msg/I79/Detection.msg"
  "msg/I79/DetectionList.msg"
  "msg/I79/Object.msg"
  "msg/I79/ObjectList.msg"
  "msg/I79/Version.msg"
  "msg/K77/DbcSignal.msg"
  "msg/K77/DbcMessage.msg"
)

# 6. Generate message code (core step)
# - Input: msg_files (message files to compile)
# - DEPENDENCIES: dependencies on other message packages (here depends on std_msgs, for Header)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS  # Optional: enable message code checks (e.g., syntax standards)
)

# 7. Export dependencies (tell other packages: depending on me also requires rosidl_default_runtime)
ament_export_dependencies(rosidl_default_runtime)

# 8. Test configuration (optional: keep if unit tests are needed)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Automatically find test dependencies (e.g., code style checks)
  ament_lint_auto_find_test_dependencies()
endif()

# 9. Declare package type (must be called, otherwise no configuration files are generated)
ament_package()
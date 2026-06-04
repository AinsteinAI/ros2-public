cmake_minimum_required(VERSION 3.8)
project(radar_subscriber)

# Adapt to Windows MSVC compiler: distinguish warning options for GCC/Clang and MSVC
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # GCC/Clang compilers: retain original warning options
  add_compile_options(-Wall -Wextra -Wpedantic)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  # MSVC compiler (Windows): replace /Wextra with /W4 to be compatible with ROS 2 warning levels
  add_compile_options(/W4 /wd4251 /wd4275)  # /wd4251 and /wd4275 are optional, used to suppress ROS 2-related warnings
endif()

# Find dependent packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(radar_msgs REQUIRED)

# Compile the subscriber node
add_executable(radar_subscriber src/radar_subscriber.cpp)
if(WIN32)
  # Path of resource file (version.rc under src directory)
  set(VERSION_RC_FILE ${CMAKE_CURRENT_SOURCE_DIR}/res/version.rc)
  # Add the resource file to the compilation dependencies of the executable
  target_sources(radar_subscriber PRIVATE ${VERSION_RC_FILE})
  # Link the libraries required for Windows resource compilation (included by default, explicit declaration is more reliable)
  target_link_libraries(radar_subscriber user32.lib)
endif()


ament_target_dependencies(radar_subscriber
  rclcpp
  radar_msgs
)

# Install the executable file
install(TARGETS
  radar_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
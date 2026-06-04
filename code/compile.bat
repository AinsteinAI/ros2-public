cd d:\ros2_ws

rmdir /s /q build\radar_muniu
rmdir /s /q install\radar_muniu
rmdir /s /q log\*

call D:\ros2_humble\local_setup.bat

colcon build --packages-select radar_msgs --event-handlers console_direct+

colcon build --packages-select radar_muniu --event-handlers console_direct+

colcon build --packages-select radar_subscriber --event-handlers console_direct+

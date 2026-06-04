# radar_muniu/launch/radar_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Locate parameter file path (Recommended approach to avoid hardcoding paths)
    params_path = PathJoinSubstitution([
        FindPackageShare("radar_muniu"),  # Package name
        "config",                         # Config file directory
        "radar_muniu_config.yaml"         # Parameter file name
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("radar_muniu"),  # Package name
        "rviz",                           # Config file directory (match your saved directory)
        "radar_view.rviz"                 # Config file name (match your saved name)
    ])

    return LaunchDescription([
        # Launch radar node
        Node(
            package="radar_muniu",         # Package name
            executable="radar_muniu_node", # Executable file name
            name="radar_muniu",            # Node name (optional, defaults to executable name)
            output="screen",               # Log output to terminal (default outputs to log file)
            #parameters=[params_path],     # Load parameter file
        ),

        # Launch filter node (Example: assume another node for processing radar data)
        #Node(
        #    package="radar_filters",
        #    executable="filter_node",
        #    name="radar_filter",
        #    output="screen",
        #    parameters=[{"filter_type": "median"}],  # Set parameters directly (instead of file)
        #    remappings=[("/input", "/radar/objects"), ("/output", "/radar/filtered_objects")]
        #),

        # Launch RViz2 (Visualization)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", PathJoinSubstitution([FindPackageShare("radar_muniu"), "rviz", "radar_view.rviz"])]
        )
    ])
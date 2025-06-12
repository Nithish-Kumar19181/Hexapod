from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os 
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # Include the view_sllidar_c1_launch.py launch file
    lidar_scan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'),
                         'launch/view_sllidar_c1_launch.py')
        ),  # optional arguments if needed
    )

    return LaunchDescription([
        lidar_scan,
        # You can add more actions or configurations as needed
    ])

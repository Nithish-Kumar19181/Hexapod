from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    joy_node = Node(
        package='joy',
        namespace='joystick',
        executable='joy_node',
        name='sai_joystick',
        output='screen'
    )

    joystick_mapper_node = Node(
        package='hexapod_controller',  # Replace with your package name
        executable='joystick_mapper',  # Replace with your node executable name
        name='joystick_mapper',
        output='screen'
    )
    return LaunchDescription([
        ExecuteProcess(
            cmd=['sudo', 'chmod', '666', '/dev/ttyUSB0'],
            shell=True,
            output='screen',
            name='set_permissions'
        ),
        joy_node,
        joystick_mapper_node,
        Node(
            package='hexapod_firmware',
            executable='simple_serial_transmitter.py',
            name='simple_serial_transmitter',
            output='screen',
            parameters=[
                {"port": "/dev/ttyUSB0"},  # Modify this if your port is different
                {"baudrate": 115200},     # Modify this if your baudrate is different
            ],
        ),
    ])

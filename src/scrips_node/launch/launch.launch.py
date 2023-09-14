from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='scrips_node' #<--- CHANGE ME

    laser = Node(
            package='scrips_node',
            executable='follow_the_gap'
            )
    
    pid_wf = Node(
            package='scrips_node',
            executable='pidWF_node'
            )
    
    stop = Node(
            package='scrips_node',
            executable='stop_node'
            )

    # Launch them all!
    return LaunchDescription([
        laser,
        pid_wf,
        stop
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Deklariere die Launch-Argumente [cite: 70]
    battery_capacity_arg = DeclareLaunchArgument(
        'battery_capacity', default_value='100.0',
        description='Initial battery capacity (percentage).'
    )
    low_battery_threshold_arg = DeclareLaunchArgument(
        'low_battery_threshold', default_value='20.0',
        description='Threshold to trigger low battery warning.'
    )
    delivery_speed_arg = DeclareLaunchArgument(
        'delivery_speed', default_value='1.0',
        description='Robot speed (units/sec).'
    )

    # Lese die Launch-Konfigurationen
    battery_capacity = LaunchConfiguration('battery_capacity')
    low_battery_threshold = LaunchConfiguration('low_battery_threshold')
    delivery_speed = LaunchConfiguration('delivery_speed')

    # Definiere die 4 Nodes [cite: 107]
    turtlesim_node = Node(
    package='turtlesim',
    executable='turtlesim_node',
    name='turtlesim'
    )

    order_server_node = Node(
        package='delivery_bot',
        executable='order_server',
        name='order_server'
    )

    navigator_node = Node(
        package='delivery_bot',
        executable='navigator',
        name='navigator',
        parameters=[
            {'battery_capacity': battery_capacity},
            {'delivery_speed': delivery_speed}
        ]
    )

    battery_manager_node = Node(
        package='delivery_bot',
        executable='battery_manager',
        name='battery_manager',
        parameters=[
            {'low_battery_threshold': low_battery_threshold}
        ]
    )

    status_display_node = Node(
        package='delivery_bot',
        executable='status_display',
        name='status_display',
        output='screen' # Zeige die Logs direkt im Terminal
    )

    return LaunchDescription([
        battery_capacity_arg,
        low_battery_threshold_arg,
        delivery_speed_arg,
        turtlesim_node,
        order_server_node,
        navigator_node,
        battery_manager_node,
        status_display_node
    ])
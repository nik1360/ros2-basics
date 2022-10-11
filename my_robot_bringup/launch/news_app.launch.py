from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_news_station_node = Node(
        package="my_py_pkg",
        executable="robot_news_station", # The one in setup.py
        name="my_news_station", # Change default node name
        remappings=[
            ("robot_news", "new_robot_news") # Topic name remap
        ],
        parameters=[    # Assign parameters value
            {"robot_name": "CIRO"},
            {"publish_period": 2}
        ]
    )

    smartphone_node = Node(
        package = "my_py_pkg",
        executable = "smartphone", 
        name="galaxy_note",
        remappings=[
            ("robot_news", "new_robot_news") 
        ]
        
    )


    ld.add_action(robot_news_station_node)
    ld.add_action(smartphone_node)

    return ld



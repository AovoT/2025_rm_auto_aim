from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = Node(
        name="image_container",
        package="rclcpp_components",
        executable="component_container",
        output="both"
    )

    load_composable_nodes = LoadComposableNodes(
        target_container="image_container",
        composable_node_descriptions=[
            ComposableNode(
                package="armor_tracker",
                plugin="armor_auto_aim::ArmorTrackerNode",
                name="armor_tracker_node"
            )
        ]
    )
    return LaunchDescription([container, load_composable_nodes])
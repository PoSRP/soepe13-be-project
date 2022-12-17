from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  ld = LaunchDescription()

  server_node = Node(
    package="ecat_server",
    executable="ecat_server",
    prefix="ethercat_grant"
  )

  ld.add_action(server_node)
  return ld

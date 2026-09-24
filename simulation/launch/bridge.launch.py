"""Launch ros_gz_bridge for Ignition Gazebo to ROS 2 communication."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            # Camera: Gazebo -> ROS 2
            "/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            # LiDAR point cloud: Gazebo -> ROS 2
            "/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            # IMU: Gazebo -> ROS 2
            "/drone/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            # GPS: Gazebo -> ROS 2
            "/drone/gps/fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat",
            # UAV Pose: Gazebo -> ROS 2
            "/model/sih_uav/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose",
            # UAV Control: ROS 2 -> Gazebo
            "/model/sih_uav/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            # Clock: Gazebo -> ROS 2
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        remappings=[
            ("/model/sih_uav/pose", "/drone/pose"),
            ("/model/sih_uav/cmd_vel", "/drone/cmd_vel"),
        ],
        output="screen",
    )

    return LaunchDescription([bridge])

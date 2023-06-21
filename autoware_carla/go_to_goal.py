import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from autoware_auto_msgs.msg import Trajectory
from autoware_auto_msgs.msg import VehicleControlCommand


class AutowareNode(Node):
    def __init__(self):
        super().__init__('autoware_example')

        # Subscribe to the goal topic
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.goal_callback,
            10
        )

        # Create a publisher for the PoseWithCovarianceStamped messages
        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.publisher_initial_pose = self.create_publisher(
            msg_type=PoseWithCovarianceStamped,
            topic='/initialpose',
            qos_profile=qos_profile)

        # Create a publisher for the PoseStamped messages
        self.publisher = self.create_publisher(
            PoseStamped,
            '/pose_stamped_topic',
            10
        )

        # Create a publisher for the planned trajectory
        self.trajectory_publisher = self.create_publisher(
            Trajectory,
            '/planned_trajectory',
            10
        )

        # Create a publisher for the vehicle control commands
        self.control_publisher = self.create_publisher(
            VehicleControlCommand,
            '/vehicle_control_cmd',
            10
        )

        self.current_goal = None

    def goal_callback(self, goal):
        # This function is called when a new goal is received
        self.current_goal = goal

    def navigate_to_goal(self):
        if self.current_goal is None:
            return

        # Your code to process the goal and plan a trajectory goes here
        # For simplicity, let's assume the planned trajectory is a straight line

        # Create a Trajectory message
        trajectory = Trajectory()

        # Create a single waypoint in the trajectory
        waypoint = PoseStamped()
        waypoint.pose.position.x = 0.0
        waypoint.pose.position.y = 0.0
        waypoint.pose.position.z = 0.0
        waypoint.pose.orientation.x = 0.0
        waypoint.pose.orientation.y = 0.0
        waypoint.pose.orientation.z = 0.0
        waypoint.pose.orientation.w = 1.0

        # Add the waypoint to the trajectory
        trajectory.points.append(waypoint)

        # Publish the planned trajectory
        self.trajectory_publisher.publish(trajectory)

        # Simulate vehicle control commands
        control_command = VehicleControlCommand()
        control_command.velocity = 10.0  # Example velocity, adjust as needed
        control_command.acceleration = 0.0
        control_command.jerk = 0.0

        self.control_publisher.publish(control_command)

    # def main(self):
    #     # Start the main ROS 2 loop
    #     while rclpy.ok():
    #         self.navigate_to_goal()

    #         rclpy.spin_once(self)

    #     # Cleanup when the node is shutting down
    #     self.destroy_node()
    #     rclpy.shutdown()

    def go_to_goal(self):
        # Create a PoseStamped message
        pose = PoseStamped()

        # Set the pose values
        pose.header.frame_id = 'map'
        pose.pose.position.x = 250.48
        pose.pose.position.y = 133.49
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        # Publish the PoseStamped message
        self.publisher.publish(pose)
        self.get_logger().info('Published PoseStamped message')

    def initialize(self):

        # Create a PoseWithCovarianceStamped message
        pose = PoseWithCovarianceStamped()

        # Set the pose values
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = 161.0
        pose.pose.pose.position.y = -133.0
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.x = 0.0
        pose.pose.pose.orientation.y = 0.0
        pose.pose.pose.orientation.z = 0.0
        pose.pose.pose.orientation.w = 1.0

        # Set the covariance values
        pose.pose.covariance = [0.0] * 36  # Fill covariance matrix with zeros
        pose.pose.covariance[0] = 0.25  # x
        pose.pose.covariance[7] = 0.25  # y
        pose.pose.covariance[35] = 0.06853891945200942  # yaw
        # Publish the PoseWithCovarianceStamped message
        self.publisher_initial_pose.publish(pose)
        self.get_logger().info('Published PoseWithCovarianceStamped message')


def main():
    rclpy.init()
    autoware_node = AutowareNode()
    autoware_node.initialize()
    # autoware_node.go_to_goal()
    # Cleanup when the node is shutting down
    autoware_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

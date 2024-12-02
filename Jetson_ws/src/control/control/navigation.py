import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from ff_interface.srv import RobotControl

class Navigation:
    def __init__(self, node: Node):
        self.node = node
        
        # Publisher for robot velocity control
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for robot position (e.g., from localization or ArUCo detection)
        self.current_pose = None
        self.pose_sub = node.create_subscription(
            PoseStamped, '/current_pose', self.pose_callback, 10
        )

        # Service server for manual robot control
        self.control_srv = node.create_service(
            RobotControl, '/RobotControl', self.manual_control_callback
        )

    def pose_callback(self, msg: PoseStamped):
        """Callback to update the robot's current position."""
        self.current_pose = msg.pose

    async def go_to(self, target_pose):
        """
        Navigate the robot to the specified target position.

        :param target_pose: Target pose as a geometry_msgs/Pose
        :return: True if successfully reached the target, otherwise False
        """
        self.node.get_logger().info("Navigating to target position...")
        
        # Basic logic to move towards the target
        rate = self.node.create_rate(10)  # 10 Hz
        while rclpy.ok():
            if not self.current_pose:
                self.node.get_logger().info("Waiting for current position data...")
                rate.sleep()
                continue
            
            # Compute distance to target
            distance = self.compute_distance(self.current_pose, target_pose)
            if distance < 0.1:  # Tolerance for reaching the target
                self.node.get_logger().info("Target reached!")
                self.stop()
                return True

            # Compute velocity command (example: proportional controller)
            velocity = self.compute_velocity(self.current_pose, target_pose)
            self.cmd_vel_pub.publish(velocity)
            rate.sleep()

        return False

    def compute_distance(self, current_pose, target_pose):
        """
        Compute the Euclidean distance between the current and target pose.

        :param current_pose: Current pose as geometry_msgs/Pose
        :param target_pose: Target pose as geometry_msgs/Pose
        :return: Distance as a float
        """
        dx = target_pose.position.x - current_pose.position.x
        dy = target_pose.position.y - current_pose.position.y
        return (dx**2 + dy**2)**0.5

    def compute_velocity(self, current_pose, target_pose):
        """
        Compute a Twist message to navigate towards the target pose.

        :param current_pose: Current pose as geometry_msgs/Pose
        :param target_pose: Target pose as geometry_msgs/Pose
        :return: Twist message
        """
        twist = Twist()
        dx = target_pose.position.x - current_pose.position.x
        dy = target_pose.position.y - current_pose.position.y
        angle_to_target = atan2(dy, dx)

        # Example simple proportional control
        twist.linear.x = min(0.2, max(0.05, self.compute_distance(current_pose, target_pose) * 0.5))
        twist.angular.z = angle_to_target
        return twist

    def stop(self):
        """Stop the robot."""
        self.node.get_logger().info("Stopping the robot...")
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def manual_control_callback(self, request, response):
        """
        Callback for /RobotControl service to manually control the robot.

        :param request: RobotControl.Request
        :param response: RobotControl.Response
        :return: RobotControl.Response
        """
        self.node.get_logger().info(f"Received manual control command: {request.command}")
        if request.command == "stop":
            self.stop()
            response.success = True
        elif request.command == "forward":
            twist = Twist()
            twist.linear.x = 0.2
            self.cmd_vel_pub.publish(twist)
            response.success = True
        elif request.command == "backward":
            twist = Twist()
            twist.linear.x = -0.2
            self.cmd_vel_pub.publish(twist)
            response.success = True
        elif request.command == "turn_left":
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
            response.success = True
        elif request.command == "turn_right":
            twist = Twist()
            twist.angular.z = -0.5
            self.cmd_vel_pub.publish(twist)
            response.success = True
        else:
            response.success = False
            response.message = "Unknown command"
        return response

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from ff_interface.action import Job  # Define your custom Job Action
from ff_interface.srv import BoxDetectandPick
from geometry_msgs.msg import Twist
from navigation import Navigation


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Action Server for /job
        self.job_action_server = ActionServer(
            self,
            Job,
            '/job',
            self.execute_job_callback
        )
        
        # Initialize navigation and manipulation instances
        self.navigation = Navigation(self)
        
        # /cmd_vel publisher for robot movement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # /ArUcoDetections subscriber for detecting destinations
        self.destination_sub = self.create_subscription(
            PoseArray,
            '/ArucoDetections',
            self.aruco_callback,
            10
        )
        
        self.destinations = {}  # Dictionary to store ArUCo marker coordinates

    def aruco_callback(self, msg):
        """Callback to update destination coordinates from /ArucoDetections."""
        for pose in msg.poses:
            self.destinations[pose.id] = pose  # Assume pose.id provides destination ID

    async def execute_job_callback(self, goal_handle):
        """Execute job action based on the received goal."""
        self.get_logger().info(f"Received job request: {goal_handle.request.job}")
        job = goal_handle.request.job.split(', ')
        
        # Step 1: Navigate to destination1
        self.get_logger().info("Navigating to destination1...")
        await self.navigation.go_to(self.destinations.get(1))  # Assuming 1 is destination1

        # Step 2-6: Handle boxes for job
        box_colors = self.parse_job_boxes(job)
        for color in box_colors:
            # Request BoxDetectandPick service
            self.get_logger().info(f"Picking up {color} box...")
            response = await self.request_box_detect_and_pick(color)
            if not response or not response.success:
                self.get_logger().error(f"Failed to pick up {color} box.")
                goal_handle.abort()
                return Job.Result(status="error", details=f"Failed to pick up {color} box.")

            # Navigate to destination2
            self.get_logger().info("Navigating to destination2...")
            await self.navigation.go_to(self.destinations.get(2))  # Assuming 2 is destination2

            # Drop the box
            self.get_logger().info("Releasing box at destination2...")
            self.manipulation.release_gripper()

        # Step 7: Navigate to destination3
        self.get_logger().info("Navigating to destination3...")
        await self.navigation.go_to(self.destinations.get(3))  # Assuming 3 is destination3

        # Step 8: Pick purple box
        self.get_logger().info("Picking up purple box...")
        response = await self.request_box_detect_and_pick("purple")
        if not response or not response.success:
            self.get_logger().error("Failed to pick up purple box.")
            goal_handle.abort()
            return Job.Result(status="error", details="Failed to pick up purple box.")

        # Step 10: Navigate to destination4 and appropriate section
        section = self.get_section_from_job(job)
        self.get_logger().info(f"Navigating to section{section}...")
        await self.navigation.go_to(self.destinations.get(4))  # Assuming 4 is destination4
        await self.navigation.go_to(self.destinations.get(section))  # Navigate to specific section

        # Job completed
        self.get_logger().info("Job completed successfully!")
        goal_handle.succeed()
        return Job.Result(status="success", job_completed=True, details=f"Completed job at section{section}.")

    def parse_job_boxes(self, job):
        """Parse the job and return the list of box colors."""
        box_colors = []
        for item in job:
            if 'red' in item or 'blue' in item:
                color, count = item[:-1], int(item[-1])
                box_colors.extend([color] * count)
        return box_colors

    def get_section_from_job(self, job):
        """Extract the section number from the job."""
        for item in job:
            if 'section' in item:
                return int(item[-1])
        return None

    async def request_box_detect_and_pick(self, color):
        """Request the BoxDetectandPick service to pick up a box."""
        client = self.create_client(BoxDetectandPick, '/BoxDetectandPick')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /BoxDetectandPick service...')
        
        request = BoxDetectandPick.Request()
        request.color = color
        future = client.call_async(request)
        response = await future
        return response


def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

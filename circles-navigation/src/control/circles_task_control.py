import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

from circles_detector import CirclesDetector
import cv2



class AutomaticController(Node):

    def __init__(self):
        super().__init__('automatic_control')
        self.vel_publisher = self.create_publisher(Twist, 'ROV/cmd_vel', 10)
        self.depth_subscriber = self.create_subscription(Float64, 'ROV/depth', self.depth_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.velocity = Twist()
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0
        self.depth = 0

        self.detector = CirclesDetector()
        self.directions = []
        self.current_circle = 0
        self.frame = None
        self.is_aligned = False

    def depth_callback(self, msg):
        """
        Callback function for depth subscription.

        Parameters:
        msg (Float64): The message containing the depth information.
        """        
        self.depth = msg.data

    def cmd(self, dx, dy, dz):
        """
        Set the velocity command for the ROV based on input directions.

        Parameters:
        dx (float): The forward/backward motion command.
                    Positive values move the ROV forward.
                    Negative values move the ROV backward.
        dy (float): The right/left motion command.
                    Positive values move the ROV to the right.
                    Negative values move the ROV to the left.
        dz (float): The depth adjustment command.
                    Positive values move the ROV downward.
                    Negative values move the ROV upward.
        """
        self.velocity.linear.x = dx * 0.01
        self.velocity.linear.y = dy * 0.01 
        self.velocity.linear.z = dz * 0.01
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0

        self.get_logger().info(str(self.velocity))


    def timer_callback(self):
        """
        Timer callback function for autonomous control.

        This function is called periodically to process the current frame, determine
        the direction to move, and update the ROV's velocity accordingly.
        """        
        if self.frame is not None:
            delta_x, delta_y, threshold_met = self.detector.determine_direction(self.frame)
            self.cmd(delta_x, delta_y, 0)
            self.vel_publisher.publish(self.velocity)

            if threshold_met:
                self.get_logger().info(f"Passing thru circle {self.current_circle}")
                self.cmd(1, 0, 0)  # move forward
                self.vel_publisher.publish(self.velocity)
                #TODO: Re-adjust the delay period based on the ROV's actual velocity 
                rclpy.spin_once(self, timeout_sec=2.0)  # move forward for a short duration

                self.get_logger().info("Passed")
                self.current_circle += 1

                # Move toward the next curcle
                if self.directions[self.current_circle] == "right":
                    self.get_logger().info(f"Moving right toward circle {self.current_circle}")
                    self.cmd(0, 1, 0)
                else:
                    self.get_logger().info(f"Moving left toward circle {self.current_circle}")
                    self.cmd(0, -1, 0)

                self.is_aligned = False
                while not self.is_aligned:
                    delta_x, delta_y, threshold_met = self.detector.determine_direction(self.frame)
                    # For x-axis (delta_x) control, the ROV moves in y
                    # For depth (delta_y) control, it moves in z 
                    self.cmd(0, delta_x, delta_y)
                    self.vel_publisher.publish(self.velocity)

                    if abs(delta_x) <= 10 and abs(delta_y) <= 10:
                        self.get_logger().info("Centered")
                        self.is_aligned = True

                if self.current_circle >= len(self.directions):
                    self.get_logger().info("Task Completed")
                    self.velocity = Twist()
                    self.vel_publisher.publish(self.velocity)
                    return                


def main(args=None):
    rclpy.init(args=args)

    controller = AutomaticController()

    #TODO: replace 0 with the front camera ip
    cap = cv2.VideoCapture(0)

    # Set directions manually (for testing purposes), real values are obtained from Shaheen's code
    sequence = ["right", "left"]
    controller.directions = sequence


    try:
        ret, frame = cap.read()
        if ret:
            controller.frame = frame
        rclpy.spin(controller)
    except KeyboardInterrupt:
        cap.release()
        controller.destroy_timer(controller.timer)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
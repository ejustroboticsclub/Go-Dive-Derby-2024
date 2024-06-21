import rclpy
from rclpy.node import Node
import dataclasses
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import ByteMultiArray
from tf_transformations import euler_from_quaternion

class MappingController(Node):

    def __init__(self):
        super().__init__('mapping_control')
        self.vel_publisher = self.create_publisher(Twist, 'ROV/cmd_vel', 10)
        self.imu = self.create_subscription(Imu, 'ROV/imu', self.imu_callback, 10)
        self.motion_triggers_subscriber = self.create_subscription(ByteMultiArray, 'ROV/mapping_buttons', self.motion_triggers_callback, 1)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.velocity = Twist()
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0
        self.orientation = 0

        # Assuming the start point is the upper right corner of the tomb
        self.rotation_sequence = ['left', 'left', 'right', 'right']
        self.rotation_counter = 0
        self.rotation_angle = 1.57  # 90 degrees 
        self.motion_triggers = ByteMultiArray()
        self.mission_interrupted = False
        self.rotation_trigger = False
        self.target_orientation = None


    def imu_callback(self, msg):
        _roll, _pith, yaw = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        )
        self.orientation = yaw
    
    def motion_triggers_callback(self, triggers_msg: ByteMultiArray):
        self.mission_interrupted = triggers_msg.data[1]
        self.rotation_trigger = triggers_msg.data[0]

    def rotate_right(self):
        self.velocity.angular.z = 1.0
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        
    
    def rotate_left(self):
        self.velocity.angular.z = -1.0
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0

    def forward_movement(self):
        self.velocity.linear.x = 1.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0

    def stop(self):
        self.velocity.linear.x = 0.0
        self.velocity.linear.y = 0.0
        self.velocity.linear.z = 0.0
        self.velocity.angular.x = 0.0
        self.velocity.angular.y = 0.0
        self.velocity.angular.z = 0.0


    def timer_callback(self):

        if not self.mission_interrupted:

            if self.rotation_trigger:

                if self.rotation_sequence[self.rotation_counter] == 'left':
                    self.target_orientation = self.orientation - self.rotation_angle
                else:
                    self.target_orientation = self.orientation + self.rotation_angle
                
                self.rotation_counter = (self.rotation_counter + 1) % len(self.rotation_sequence)

            # Rotate to the designated direction with a tolerance of 5 degrees
            if abs(self.orientation - self.target_orientation) > 0.087:

                if self.rotation_sequence[self.rotation_counter - 1] == 'left':
                    self.rotate_left()
                else:
                    self.rotate_right()
            else: 
                self.target_orientation = self.orientation
                self.forward_movement()

                



        # if mission is interrupted, stop
        else:
            self.stop()

        self.vel_publisher.publish(self.velocity)
        self.get_logger().info(str(self.velocity))



def main(args=None):
    rclpy.init(args=args)

    controller = MappingController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        # controller.get_logger().info('Shutting down...')
        controller.destroy_timer(controller.timer)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
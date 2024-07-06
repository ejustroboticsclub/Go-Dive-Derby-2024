import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool




class ShapeDetectorNode(Node):
    def __init__(self):
        super().__init__("shape_detection")
        self.path = "/home/atef/shape.txt"        
        #self.file = open(self.path)

        self.cube_pub = self.create_publisher(Bool, "Cube", 10)
        self.cuboid_pub = self.create_publisher(Bool, "Cuboid", 10)
        self.pipe_pub = self.create_publisher(Bool, "pipe", 10)
        self.check_button = False
        self.topic = None
        
        self.my_publishers = {
            "Cube": self.cube_pub,
            "Cuboid": self.cuboid_pub,
            "pipe": self.pipe_pub
        }

        self.subscription = self.create_subscription(
            Bool,
            '/ROV/shape',
            self.shape_callback,
            10
        )

        self.timer = self.create_timer(0.1,self.timer_callback)

    def shape_callback(self, msg):
        self.check_button = msg.data

    def timer_callback(self):
        #self.topic = self.file.read()
        with open(self.path, 'r') as f:
             self.topic = f.read()

        print(self.topic)
        if (self.topic != "None") and self.check_button:
            pub = self.my_publishers.get(self.topic)
            if pub:
                msg = Bool()
                msg.data = True
                pub.publish(msg)




def main(args=None):
        rclpy.init(args=args)
        shape_detector_node = ShapeDetectorNode()
        rclpy.spin(shape_detector_node)
        shape_detector_node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

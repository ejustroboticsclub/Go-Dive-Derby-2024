import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from imu_msgs.msg import Angle 
from std_msgs.msg import Bool
import pygame
import math
import tf

# Initialize pygame
pygame.init()

# Pool and ROV parameters
POOL_WIDTH_M = 5.0
POOL_HEIGHT_M = 5.0
ACTUAL_ROV_SPEED_MPS = 0.1

# Pygame display settings
PIXELS_PER_METER = 200
WIDTH, HEIGHT = int(POOL_WIDTH_M * PIXELS_PER_METER), int(POOL_HEIGHT_M * PIXELS_PER_METER)
SCALE_WIDTH = 200

screen = pygame.display.set_mode((WIDTH + SCALE_WIDTH, HEIGHT))
pygame.display.set_caption("ROV Track Visualization")

background_image = pygame.image.load('pool.jpg')
background_image = pygame.transform.scale(background_image, (WIDTH, HEIGHT))
rov_image = pygame.image.load('rov.png')
rov_image = pygame.transform.scale(rov_image, (60, 60))
Cuboid_image = pygame.image.load('cuboid.jpg')
Cuboid_image = pygame.transform.scale(Cuboid_image, (40, 40))
Cube_image = pygame.image.load('cube.png')
Cube_image = pygame.transform.scale(Cube_image, (40, 40))
pipe_image = pygame.image.load('pipe.jpg')
pipe_image = pygame.transform.scale(pipe_image, (40, 40))

# Colors
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

class Mapping():
    def __init__(self) -> None:
        rospy.init_node('rov_visualizer', anonymous=True)
        rospy.Subscriber('ROV/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.Subscriber('/imu/angle', Angle, self.imu_callback)
        rospy.Subscriber('Cube', Bool, self.Cube_callback)
        rospy.Subscriber('Cuboid', Bool, self.Cuboid_callback)
        rospy.Subscriber('pipe', Bool, self.pipe_callback)
        
        self.initial_yaw_angle = None
        
        # Time step
        self.dt = 0.01

        self.rate = rospy.Rate(1/self.dt)
        self.clock = pygame.time.Clock()
    
        # Start position and orientation
        self.x = WIDTH // 2
        self.y = HEIGHT // 2

        self.roll = 0 # Facing upward
        self.pitch = 0
        self.yaw = 0       

        # Initialize velocities
        self.linear_velocity_x = 0
        self.linear_velocity_y = 0
        self.angular_velocity = 0

        # Path and pin markers
        self.path = [(self.x, self.y)]
        self.cube = []
        self.pipe = []
        self.cuboid = []

    def scale_velocity(self):
        """Scale velocity from [-1, 1] to actual speed in pixels per second."""
        vx_scaled = self.linear_velocity_x * ACTUAL_ROV_SPEED_MPS * PIXELS_PER_METER
        vy_scaled = self.linear_velocity_y * ACTUAL_ROV_SPEED_MPS * PIXELS_PER_METER
        return vx_scaled, vy_scaled

    def update_position(self):
        """Update the position of the ROV based on velocities and time step."""
        vx_scaled, vy_scaled = self.scale_velocity()
        self.x += vx_scaled * self.dt * math.cos(math.radians(self.yaw)) - vy_scaled * self.dt * math.sin(math.radians(self.yaw))   
        self.y += vx_scaled * self.dt * math.sin(math.radians(self.yaw)) + vy_scaled * self.dt * math.cos(math.radians(self.yaw))
        return self.x, self.y

    def cmd_vel_callback(self, data):
        """Callback for receiving velocity commands."""
        self.linear_velocity_x = data.linear.x
        self.linear_velocity_y = data.linear.y
        self.angular_velocity = data.angular.z

    def imu_callback(self, data):
        """Callback for receiving IMU data and extracting the yaw angle."""
        if self.initial_yaw_angle is None or self.initial_yaw_angle == 100.0:
            self.initial_yaw_angle = data.yaw
        self.roll = data.roll
        self.pitch = data.pitch
        self.yaw = -(data.yaw - self.initial_yaw_angle)
        print(self.yaw)

    def pipe_callback(self, data):
        """Callback for detecting and marking pipes."""
        if data.data:
            self.pipe.append((self.x, self.y))

    def Cube_callback(self, data):
        """Callback for detecting and marking cubes."""
        if data.data:
            self.cube.append((self.x, self.y))

    def Cuboid_callback(self, data):
        """Callback for detecting and marking cuboids."""
        if data.data:
            self.cuboid.append((self.x, self.y))

    def draw_scale(self):
        """Draw the scale on the right side of the screen."""
        scale_surface = pygame.Surface((SCALE_WIDTH, HEIGHT))
        scale_surface.fill(WHITE)
        
        font = pygame.font.SysFont('Arial', 20)
        for i in range(int(POOL_HEIGHT_M) + 1):
            y_pos = HEIGHT - int(i * PIXELS_PER_METER)
            pygame.draw.line(scale_surface, BLACK, (0, y_pos), (20, y_pos), 2)
            label = font.render(f'{i}m', True, BLACK)
            scale_surface.blit(label, (25, y_pos - 10))

        screen.blit(scale_surface, (WIDTH, 0))

def main():
    Map = Mapping()

    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.signal_shutdown('Quit event received')

        Map.x, Map.y = Map.update_position()
        
        Map.path.append((Map.x, Map.y))
       
        screen.blit(background_image, (0, 0))
        
        for i in range(len(Map.path) - 1):
            pygame.draw.line(screen, RED, Map.path[i], Map.path[i+1], 8)
        
        rotated_rov_image = pygame.transform.rotate(rov_image, -Map.yaw)
        rov_rect = rotated_rov_image.get_rect(center=(int(Map.x), int(Map.y)))
        screen.blit(rotated_rov_image, rov_rect.topleft)
        
        for pin in Map.cube:
            pin_rect = Cube_image.get_rect(center=pin)
            screen.blit(Cube_image, pin_rect.topleft)
        
        for pin in Map.pipe:
            pin_rect = pipe_image.get_rect(center=pin)
            screen.blit(pipe_image, pin_rect.topleft)
        
        for pin in Map.cuboid:
            pin_rect = Cuboid_image.get_rect(center=pin)
            screen.blit(Cuboid_image, pin_rect.topleft)
        
        Map.draw_scale()
        
        pygame.display.flip()
        Map.clock.tick(1/Map.dt)
        Map.rate.sleep()

    pygame.quit()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

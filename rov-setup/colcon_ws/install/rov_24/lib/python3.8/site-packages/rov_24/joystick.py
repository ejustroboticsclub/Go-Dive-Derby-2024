import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import os

import pygame
import time


class Controller(object):
    STICK_DEADBAND = 0.1

    def __init__(self, axis_map):
        self.joystick = None
        self.axis_map = axis_map

    def update(self):
        pygame.event.pump()


    def getThrottle(self):
        return self._getAxis(0)

    def getRoll(self):
        return self._getAxis(1)

    def getPitch(self):
        return self._getAxis(2)

    def getYaw(self):
        return self._getAxis(3)

    def _getAxis(self, k):
        j = self.axis_map[k]
        val = self.joystick.get_axis(abs(j))
        if abs(val) < Controller.STICK_DEADBAND:
            val = 0
        return (-1 if j < 0 else +1) * val

class _GameController(Controller):
    def __init__(self, axis_map, button_id):
        Controller.__init__(self, axis_map)
        
        self.button_id = button_id
        self.button_is_down = False
        self.switch_value = -1
        self.depth = 0
        self.right_gripper = False
        self.left_gripper = False
        self.left_gripper_button_is_down = False
        self.right_gripper_button_is_down = False
        self.auto_button_is_down = False
        self.auto_b = False


    def _getAuxValue(self):
        return self.joystick.get_button(self.button_id)

    def getTrigger(self):
        if self._getAuxValue():
            if not self.button_is_down:
                self.switch_value = True
            self.button_is_down = True
        else:
            self.switch_value = False
            self.button_is_down = False
        return self.switch_value

    def getAimball(self):
        """Reads the value of the small ball (hat) on the top

        Returns:
            x and y state (tuple): the active directions of x and y (-1 or 1) 
        """
        return self.joystick.get_hat(0)
    
    def depth_v(self):
        # if self.joystick.get_button(4):
        #     self.depth += 0.1
        # elif self.joystick.get_button(5):
        #     self.depth -= 0.1
        if self.joystick.get_button(4):
            self.depth += 0.12
        elif self.joystick.get_button(5):
            self.depth -= 0.12
        else:
            self.depth = 0.0

        self.depth = max(-1, self.depth)
        self.depth = min(1, self.depth)

    def leftGripper(self):
        # Don't rise up if the maximum value reached
        if self.joystick.get_button(2):
            if not self.left_gripper_button_is_down:
                self.left_gripper = not self.left_gripper
            self.left_gripper_button_is_down = True
        else:
            self.left_gripper_button_is_down = False

    def rightGripper(self):
        # Don't rise up if the maximum value reached
        if self.joystick.get_button(3):
            if not self.right_gripper_button_is_down:
                self.right_gripper = not self.right_gripper
            self.right_gripper_button_is_down = True
        else:
            self.right_gripper_button_is_down = False

    def stopAll(self):
        # Breaks button
        return bool(self.joystick.get_button(1))
    
    def auto(self):
        if self.joystick.get_button(6):
            if not self.auto_button_is_down:
                self.auto_b = not self.auto_b
            self.auto_button_is_down = True
        else:
            self.auto_button_is_down = False
        return self.auto_b
    
class _SpringyThrottleController(_GameController):
    def __init__(self, axis_map, button_id):
        _GameController.__init__(self, axis_map, button_id)

        self.throttleval = -1

        self.prevtime = 0

    def getThrottle(self):
        currtime = time.time()

        # Scale throttle increment by time difference from last update
        self.throttleval += self._getAxis(0) * (currtime - self.prevtime)

        # Constrain throttle to [-1,+1]
        self.throttleval = min(max(self.throttleval, -1), +1)

        self.prevtime = currtime

        return self.throttleval


class _RcTransmitter(Controller):
    def __init__(self, axis_map, aux_id):
        Controller.__init__(self, axis_map)
        self.aux_id = aux_id

    def getAux(self):
        return +1 if self.joystick.get_axis(self.aux_id) > 0 else -1


class _Xbox360(_SpringyThrottleController):
    def __init__(self, axes, aux):
        _SpringyThrottleController.__init__(self, axes, None)

        self.aux = aux

    def _getAuxValue(self):
        return self.joystick.get_axis(self.aux) < -0.5


class _Playstation(_SpringyThrottleController):
    def __init__(self, axes):
        _SpringyThrottleController.__init__(self, axes, 7)


# Different OSs have different names for the same controller, so we don't
# need to check OS when setting up the axes.
controllers = {
    "Controller (Rock Candy Gamepad for Xbox 360)": _Xbox360((-1, 4, -3, 0), 2),
    "Rock Candy Gamepad for Xbox 360": _Xbox360((-1, 3, -4, 0), 5),
    "2In1 USB Joystick": _Playstation((-1, 2, -3, 0)),
    "Wireless Controller": _Playstation((-1, 2, -3, 0)),
    "MY-POWER CO.,LTD. 2In1 USB Joystick": _Playstation((-1, 2, -3, 0)),
    "Sony Interactive Entertainment Wireless Controller": _Playstation((-1, 3, -4, 0)),
    "Logitech Extreme 3D": _GameController((-2, 0, -1, 3), 0),
    "Logitech Logitech Extreme 3D": _GameController((-3, 0, -1, 2), 0),
    "Logitech Extreme 3D pro": _GameController((-3, 0, -1, 2), 0),
    "FrSky Taranis Joystick": _RcTransmitter((0, 1, 2, 5), 3),
    "FrSky FrSky Taranis Joystick": _RcTransmitter((0, 1, 2, 3), 5),
    "SPEKTRUM RECEIVER": _RcTransmitter((1, 2, 5, 0), 4),
    "Horizon Hobby SPEKTRUM RECEIVER": _RcTransmitter((1, 2, 3, 0), 4),
}


def get_controller():
    # Initialize pygame for joystick support
    pygame.display.init()
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Find your controller
    controller_name = joystick.get_name()
    if not controller_name in controllers.keys():
        print("Unrecognized controller: %s" % controller_name)
        exit(1)
    controller = controllers[controller_name]
    controller.joystick = joystick

    return controller






class JoyStickNode(Node):
    def __init__(self):
        super().__init__("joystick")

        self.joystick_publisher = self.create_publisher(Twist, "ROV/joystick", 10)
        self.gripper_r_publisher = self.create_publisher(Bool, "ROV/gripper_r", 1)
        self.gripper_l_publisher = self.create_publisher(Bool, "ROV/gripper_l", 1)
        self.depth_stable_publisher = self.create_publisher(Bool, "ROV/depth_stable", 1)
        
        self.initial_time = None
        self.delay_flag = False
        self.shape_publisher = self.create_publisher(Bool, "/ROV/shape", 1)

        self.timer = self.create_timer(0.1, self.update)
        self.controller = get_controller()

        self.twist_msg = Twist()
        self.gripper_r_msg = Bool()
        self.gripper_l_msg = Bool()
        self.depth_stable_msg = Bool()
        self.place_bool_msg = Bool()
        self.shape_msg = Bool()

    def update(self):
        self.controller.update()
        if not self.controller.auto():
            aim_ball_x = self.controller.getAimball()[0]
            aim_ball_y = self.controller.getAimball()[1]

            # Get depth value readings
            self.controller.depth_v()

            self.controller.leftGripper()
            self.controller.rightGripper()

            self.twist_msg.linear.x = (
                float(self.controller.getPitch()) if not aim_ball_x else 0.0
            )
            self.twist_msg.linear.y = (
                float(self.controller.getRoll()) if not aim_ball_y else 0.0
            )
            self.twist_msg.linear.z = float(self.controller.depth)
            self.twist_msg.angular.z = (
                float(self.controller.getYaw())
                if not aim_ball_x and not aim_ball_y
                else 0.0
            )

            self.depth_stable_msg.data = self.controller.getTrigger()
            self.gripper_r_msg.data = self.controller.right_gripper
            self.gripper_l_msg.data = self.controller.left_gripper

            self.gripper_r_publisher.publish(self.gripper_r_msg)
            self.gripper_l_publisher.publish(self.gripper_l_msg)

            if self.controller.stopAll():
                self.shape_msg.data = True
            else:
                self.shape_msg.data = False

            self.joystick_publisher.publish(self.twist_msg)
            self.depth_stable_publisher.publish(self.depth_stable_msg)

            self.shape_publisher.publish(self.shape_msg)
        else:
            if self.initial_time is None:
                self.initial_time = time.time()
                self.delay_flag = True

            time_now = time.time()
            if time_now - self.initial_time > 5.0:
                self.delay_flag = False
                self.initial_time = None
                #self.twist_msg.linear.x = 0
                self.joystick_publisher.publish(self.twist_msg)    

            
            if self.delay_flag:
                self.twist_msg.linear.x = 1.0
                self.joystick_publisher.publish(self.twist_msg)    






def main(args=None):
    rclpy.init(args=args)
    
    jotstick_node = JoyStickNode()
    rclpy.spin(jotstick_node)

    jotstick_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
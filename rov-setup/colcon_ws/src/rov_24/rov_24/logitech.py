import pygame
import time

from joystick import Controller, _GameController


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



controllers = {
    "Logitech Extreme 3D": _GameController((-2, 0, -1, 3), 0),
    "Logitech Logitech Extreme 3D": _GameController((-3, 0, -1, 2), 0),
    "Logitech Extreme 3D pro": _GameController((-3, 0, -1, 2), 0),

}

joystick = get_controller()
while True:
    print(joystick.getYaw())
    joystick.update()
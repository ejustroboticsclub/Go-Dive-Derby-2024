import dataclasses
from time import time
import math
from typing import List
import rclpy
import rclpy.logging
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy.parameter
from std_msgs.msg import Float64, Int32MultiArray, Bool
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from rcl_interfaces.msg import SetParametersResult

PARAMETERS = [
    "x.kp",
    "x.ki",
    "x.kd",
    "y.kp",
    "y.ki",
    "y.kd",
    "w.kp",
    "w.ki",
    "w.kd",
    "depth.kp",
    "depth.ki",
    "depth.kd",
    "roll.kp",
    "roll.ki",
    "roll.kd",
    "floatability",
]

CTRL = ["pid_depth_control", "p_depth_control", "imu_control", "pid_yaw_control", "p_yaw_control"]


@dataclasses.dataclass
class ErrorVal:
    """PID error terms"""

    e_sum: float = 0
    d_error: float = 0
    current_error: float = 0
    prev_error: float = 0


class PID:
    """PID controller Class"""

    def __init__(
        self,
        k_proportaiol: float,
        k_integral: float,
        k_derivative: float,
        windup_val: float,
    ) -> None:
        """Creates a PID controller using provided PID parameters

        Args:
            k_proportaiol (float): the proportional error constant of the desired PID
            k_integral (float): the Integral error constant of the desired PID
            k_derivative (float): the derivative error constant of the desired PID
        """
        self.k_proportaiol = k_proportaiol
        self.k_integral = k_integral
        self.k_derivative = k_derivative
        self.pid: float = 0
        self.error = ErrorVal()
        self.windup_val = windup_val

    def compute(self, ref: float, measured: float) -> float:
        """Computes the PID value based on the given reference and measured output values

        Args:
            ref (float): reference signal that we desire to track
            measured (float): actual measured output of the signal

        Returns:
            float: PID value
        """
        self.error.current_error = ref - measured
        self.error.e_sum += self.error.current_error
        self.error.d_error = self.error.current_error - self.error.prev_error
        self.pid = (
            self.k_proportaiol * self.error.current_error
            + self.k_integral * self.error.e_sum
            + self.k_derivative * self.error.d_error
        )
        self.error.prev_error = self.error.current_error
        if self.error.current_error <= self.windup_val:
            self.error.e_sum = 0
        return self.pid

    def set_pid(
        self, k_proportaiol: float, k_integral: float, k_derivative: float
    ) -> None:
        """Sets the PID controller constants

        Args:
            k_proportaiol (float): the proportional error constant of the desired PID
            k_integral (float): the Integral error constant of the desired PID
            k_derivative (float): the derivative error constant of the desired PID
        """
        self.k_proportaiol = k_proportaiol
        self.k_integral = k_integral
        self.k_derivative = k_derivative


def map_from_to(
    num: float, in_min: float, in_max: float, out_min: float, out_max: float
) -> float:
    """Mapping function

    Args:
        num (float): Value to be mapped
        in_min (float): Minimum value of input
        in_max (float): Maximum value of input
        out_min (float): Minimum value of ouput
        out_max (float): Maximum value of ouput

    Returns:
        float: Mapped value
    """
    return (
        float(num - in_min) / float(in_max - in_min) * (out_max - out_min)
    ) + out_min


@dataclasses.dataclass
class Param:
    """Tuning and utils params"""

    kp_x: float
    ki_x: float
    kd_x: float
    kp_y: float
    ki_y: float
    kd_y: float
    kp_w: float
    ki_w: float
    kd_w: float
    kp_depth: float
    ki_depth: float
    kd_depth: float
    kp_roll: float
    ki_roll: float
    kd_roll: float
    floatability: float

    imu_control: bool = True
    p_depth_control: bool = True
    pid_depth_control: bool = False

    p_yaw_control: bool = True
    pid_yaw_control: bool = False

    windup_val_x = 0.1
    windup_val_y = 0.1
    windup_val_w = 0.1
    windup_val_depth = 0.1
    windup_val_roll = 0.1

    thruster_max = 1680.0
    thruster_min = 1290.0  # Changed to 1180 instead of 1160 to make sure that the center is 1485 which is a stoping value

    thruster_side_max = 1680.0
    thruster_side_min = 1290.0

    # corredponding to motion control node
    # TODO: adjust the PARAM.min and PARAM.max to all 3 motions
    max_vx = 3
    min_vx = -3
    max_vy = 3
    min_vy = -3
    max_wz = 0.7
    min_wz = -0.7
    max_pool_depth = 1
    max_roll = 90
    min_roll = -90

    accel_x_offset = 0
    accel_y_offset = 0
    accel_z_offset = 0
    angular_offset = 0


@dataclasses.dataclass
class Measured:
    """Sensor measurments"""

    v_x = 0
    v_y = 0
    w_z = 0
    yaw = 0
    v_z = 0
    roll = 0
    depth = 0


@dataclasses.dataclass
class Reference:
    """desired values"""

    v_x = 0
    v_y = 0
    w_z = 0
    v_z = 0
    roll = 0
    yaw = 0
    depth = 0


@dataclasses.dataclass
class Time:
    """Time related variables for integration and differentiation"""

    t_prev = 0
    t_current = 0
    delta_t = 0


def create_pid_objects(parameter_object: Param):
    pid_vx = PID(
        k_proportaiol=parameter_object.kp_x,
        k_derivative=parameter_object.kd_x,
        k_integral=parameter_object.ki_x,
        windup_val=parameter_object.windup_val_x,
    )
    pid_vy = PID(
        k_proportaiol=parameter_object.kp_y,
        k_derivative=parameter_object.kd_y,
        k_integral=parameter_object.ki_y,
        windup_val=parameter_object.windup_val_y,
    )
    pid_wz = PID(
        k_proportaiol=parameter_object.kp_w,
        k_derivative=parameter_object.kd_w,
        k_integral=parameter_object.ki_w,
        windup_val=parameter_object.windup_val_w,
    )
    pid_depth = PID(
        k_proportaiol=parameter_object.kp_depth,
        k_derivative=parameter_object.kd_depth,
        k_integral=parameter_object.ki_depth,
        windup_val=parameter_object.windup_val_depth,
    )
    pid_stabilization = PID(
        k_proportaiol=parameter_object.kp_roll,
        k_derivative=parameter_object.kd_roll,
        k_integral=parameter_object.ki_roll,
        windup_val=parameter_object.windup_val_roll,
    )

    return pid_vx, pid_vy, pid_wz, pid_depth, pid_stabilization


class Robot:
    """Robot Class to solve the kinematic model"""

    @staticmethod
    def kinematic_control(pid_val_x: float, pid_val_y: float, pid_val_w: float) -> List:
        """_summary_

        Args:
            pid_val_x (float): PID value calculated for the error in x velocity
            pid_val_y (float): PID value calculated for the error in y velocity
            pid_val_w (float): PID value calculated for the error in w velocity

        Returns:
            List: List of four thrusters values responsible for the planner kinematic
            motion of the ROV
        """
        # Planer Control
        phi_1 = pid_val_x - pid_val_y
        phi_2 = pid_val_x + pid_val_y
        phi_3 = pid_val_x - pid_val_y
        phi_4 = pid_val_x + pid_val_y

        # Rotation Control
        phi_1 += -pid_val_w
        phi_2 += pid_val_w
        phi_3 += pid_val_w
        phi_4 += -pid_val_w

        return [phi_1, phi_2, phi_3, phi_4]

    @staticmethod
    def find_phi_boudary_values(param: Param) -> List:
        """Estimate the thrusters boundary value solely based on max allowed velocity
            in each axis

        Args:
            param (PARAM): PARAM data class

        Returns:
            List: Boundary values for the thrusters actuation
        """
        max_phi_val = param.max_vx + param.max_vy + param.max_wz
        min_phi_val = param.min_vx + param.min_vy + param.min_wz
        return [min_phi_val, max_phi_val]

    def saturate_actuators(
        self,
        phi_1: float,
        phi_2: float,
        phi_3: float,
        phi_4: float,
        min_phi_val: float,
        max_phi_val: float,
    ) -> List:
        """_summary_

        Args:
            phi_1 (float): Unsaturated thruster 1 value in the ROV configuration
            phi_2 (float): Unsaturated thruster 2 value in the ROV configuration
            phi_3 (float): Unsaturated thruster 3 value in the ROV configuration
            phi_4 (float): Unsaturated thruster 4 value in the ROV configuration
            min_phi_val (float): Reverse saturation value
            max_phi_val (float): Forward saturation value

        Returns:
            List: Saturated list of four thrusters values responsible for the planner kinematic
            motion of the ROV
        """

        phi_1 = max(phi_1, min_phi_val)
        phi_1 = min(phi_1, max_phi_val)
        phi_2 = max(phi_2, min_phi_val)
        phi_2 = min(phi_2, max_phi_val)
        phi_3 = max(phi_3, min_phi_val)
        phi_3 = min(phi_3, max_phi_val)
        phi_4 = max(phi_4, min_phi_val)
        phi_4 = min(phi_4, max_phi_val)
        return [phi_1, phi_2, phi_3, phi_4]


class CalibrationNode(Node):
    def __init__(self):
        self.t = Time()
        self.actual = Measured()
        self.desired = Reference()

        self.t.t_prev = time()

        super().__init__("kinematic_model")

        self.thrusters_voltages_publisher = self.create_publisher(
            Int32MultiArray, "/ROV/thrusters", 10
        )
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, "/ROV/cmd_vel", self.cmd_vel_recieved_callback, 10
        )
        self.depth_subscriber = self.create_subscription(
            Float64, "/ROV/depth", self.depth_recieved_callback, 10
        )
        self.imu_subscriber = self.create_subscription(
            Imu, "/ROV/imu", self.imu_recieved_callback, 10
        )
        self.depth_stable_subscriber = self.create_subscription(
            Bool, "/ROV/depth_stable", self.depth_stable_recieved_callback, 10
        )

        self.depth_pid_subscriber = self.create_subscription(
            Twist, "/ROV/depth_pid", self.depth_pid, 10
        )

        self.yaw_pid_subsriber = self.create_subscription(
            Twist, "/ROV/yaw_pid", self.yaw_pid, 10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.ROV = Robot()

        self.declare_parameters(
            namespace="",
            parameters=[(i, rclpy.Parameter.Type.DOUBLE) for i in PARAMETERS],
        )

        self.declare_parameters(
            namespace="",
            parameters=[(i, rclpy.Parameter.Type.BOOL) for i in CTRL],
        )

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.parameters_dict = {
            i: self.get_parameter(i).get_parameter_value().double_value
            for i in PARAMETERS
        }

        for i in CTRL:
            self.parameters_dict[i] = (
                self.get_parameter(i).get_parameter_value().bool_value
            )

        self.PARAM = Param(*self.parameters_dict.values())

        (
            self.pid_vx,
            self.pid_vy,
            self.pid_wz,
            self.pid_depth,
            self.pid_stabilization,
        ) = create_pid_objects(self.PARAM)

        self.depth_flag = False
        self.depth_control_value = None
        self.yaw_control_value = None

        self.wz_flag = False
        self.roll_stbl = True

    def log_parameters(self, parameters_dict: dict):
        self.get_logger().info("Logging received parameters...")
        for param, param_value in parameters_dict.items():

            self.get_logger().info(f"{param}: {param_value}")

        self.get_logger().info("Logging parameters done...")

    def parameters_callback(self, params):
        for param in params:
            name = vars(param)["_name"]
            value = vars(param)["_value"]
            setattr(self.PARAM, name, value)

        self.parameters_dict = {
            i: self.get_parameter(i).get_parameter_value().double_value
            for i in PARAMETERS
        }

        for i in CTRL:
            self.parameters_dict[i] = (
                self.get_parameter(i).get_parameter_value().bool_value
            )

        self.log_parameters(self.parameters_dict)

        (
            self.pid_vx,
            self.pid_vy,
            self.pid_wz,
            self.pid_depth,
            self.pid_stabilization,
        ) = create_pid_objects(self.PARAM)

        return SetParametersResult(successful=True)

    def timer_callback(self):
        v_x = self.desired.v_x * self.PARAM.kp_x
        v_y = self.desired.v_y * self.PARAM.kp_y

        if self.get_parameter("pid_depth_control").get_parameter_value().bool_value:
            if self.desired.v_z == 0:
                v_z = self.depth_control_value
            else:
                v_z = self.desired.v_z

        elif self.get_parameter("p_depth_control").get_parameter_value().bool_value:
            if self.desired.v_z == 0.0:
                v_z = (self.desired.depth - self.actual.depth) * self.PARAM.kp_depth

            else:
                v_z = self.desired.v_z
        else:
            v_z = self.desired.v_z

        if self.get_parameter("imu_control").get_parameter_value().bool_value:
            if self.roll_stbl:
                stabilization_actuation = self.pid_stabilization.compute(
                    self.desired.roll, self.actual.roll
                )
            else:
                stabilization_actuation = 0

            if self.get_parameter("pid_yaw_control").get_parameter_value().bool_value:
                if self.desired.w_z == 0:
                    w_z = self.yaw_control_value
                else:
                    w_z = self.desired.w_z

            elif self.get_parameter("p_yaw_control").get_parameter_value().bool_value:
                if self.desired.w_z == 0.0:
                    w_z = (self.desired.yaw - self.actual.yaw) * self.PARAM.floatability
                    abs_error = abs(self.desired.yaw - self.actual.yaw)
                    if abs_error >= 180:
                        if (self.desired.yaw - self.actual.yaw) > 0:
                            w_z = (-1) * ((360 - abs_error) * self.PARAM.floatability)
                        else:
                            w_z = ((360 - abs_error) * self.PARAM.floatability)
                
                else:
                    w_z = self.pid_wz.compute(self.desired.w_z, self.actual.w_z)
                    
        else:
            w_z = self.desired.w_z * 5
            self.roll_control_value = 0

        phi_1, phi_2, phi_3, phi_4 = self.ROV.kinematic_control(v_x, v_y, w_z)
        min_phi_val, max_phi_val = self.ROV.find_phi_boudary_values(self.PARAM)

        phi_1, phi_2, phi_3, phi_4 = self.ROV.saturate_actuators(
            phi_1,
            phi_2,
            phi_3,
            phi_4,
            min_phi_val,
            max_phi_val,
        )

        phi_1 = map_from_to(
            phi_1,
            min_phi_val,
            max_phi_val,
            self.PARAM.thruster_min,
            self.PARAM.thruster_max,
        )
        phi_2 = map_from_to(
            phi_2,
            min_phi_val,
            max_phi_val,
            self.PARAM.thruster_min,
            self.PARAM.thruster_max,
        )
        phi_3 = map_from_to(
            phi_3,
            min_phi_val,
            max_phi_val,
            self.PARAM.thruster_min,
            self.PARAM.thruster_max,
        )
        phi_4 = map_from_to(
            phi_4,
            min_phi_val,
            max_phi_val,
            self.PARAM.thruster_min,
            self.PARAM.thruster_max,
        )

        depth_thruster = map_from_to(
            v_z,
            -1,
            1,
            self.PARAM.thruster_side_min,
            self.PARAM.thruster_side_max,
        )

        # self.get_logger().info("stable act: " + str(stabilization_actuation))

        phi_5 = depth_thruster - stabilization_actuation
        phi_6 = depth_thruster + stabilization_actuation

        # phi_5 = depth_thruster
        # phi_6 = depth_thruster

        phi_5 = max(phi_5, self.PARAM.thruster_side_min)
        phi_6 = max(phi_6, self.PARAM.thruster_side_min)
        phi_5 = min(phi_5, self.PARAM.thruster_side_max)
        phi_6 = min(phi_6, self.PARAM.thruster_side_max)

        # create the message instance
        thrusters_voltages = Int32MultiArray()

        # fill the message with the phi values
        # thrusters_voltages.data = planer_thrusters_list
        thrusters_voltages.data = [
            int(phi_1),
            int(phi_2),
            int(phi_3),
            int(phi_4),
            int(phi_5),
            int(phi_6),
        ]
        # publish the message
        self.get_logger().info(str(list(thrusters_voltages.data)))
        self.get_logger().info(
            f"actual depth: {self.actual.depth},\ndesired depth: {self.desired.depth}"
        )
        self.get_logger().info(
            f"actual w_z: {self.actual.w_z},\nroll: {self.actual.roll}, \nyaw: {self.actual.yaw}, \ndesired yaw: {self.desired.yaw}"
        )  # Check degree or radian

        self.thrusters_voltages_publisher.publish(thrusters_voltages)

    def cmd_vel_recieved_callback(self, twist_msg: Twist):
        """callback function for the subscriber to the ROV/cmd_vel topic

        Args:
            msg (Twist): Twist message sent by the control node
        """
        self.desired.v_x = twist_msg.linear.x
        self.desired.v_y = twist_msg.linear.y
        self.desired.v_z = twist_msg.linear.z
        if self.desired.v_z == 0.0:
            if not self.depth_flag:
                self.desired.depth = self.actual.depth
                self.depth_flag = True
        else:
            self.depth_flag = False

        self.desired.w_z = twist_msg.angular.z
        if self.desired.w_z == 0.0:
            if not self.wz_flag:
                self.desired.yaw = self.actual.yaw
                self.wz_flag = True
        else:
            self.wz_flag = False

    def depth_recieved_callback(self, depth_msg: Float64):
        """callback function for the subscriber to the ROV/depth topic

        Args:
            msg (Float64): depth sent by the the depth sensor
        """
        # return
        self.actual.depth = depth_msg.data

    def imu_recieved_callback(self, imu: Imu):
        """recieves the imu readings from the MPU9025
        Args:
            imu (sensor_msgs/Imu): sensor message containing linear accelerations and angular velocities
        """
        _roll_dot, _pitch_dot, yaw_dot = [
            imu.angular_velocity.x,
            -imu.angular_velocity.y,
            -imu.angular_velocity.z,
        ]
        roll, _pith, yaw = euler_from_quaternion(
            (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        )

        self.actual.w_z = yaw_dot
        self.actual.yaw = -yaw * 180 / math.pi
        self.actual.roll = roll * 180 / math.pi

    def depth_stable_recieved_callback(self, depth_stable: Bool):
        self.roll_stbl = depth_stable.data

    def depth_pid(self, twist_msg: Twist):
        self.depth_control_value = twist_msg.linear.z

    def yaw_pid(self, twist_msg: Twist):
        self.yaw_control_value = twist_msg.angular.z

    def stop_all(self):
        thrusters_voltages = Int32MultiArray()
        thrusters_voltages.data = [1485, 1485, 1485, 1485, 1485, 1485]
        self.thrusters_voltages_publisher.publish(thrusters_voltages)


def main(args=None):
    rclpy.init(args=args)

    motion_control_node = CalibrationNode()
    rclpy.spin(motion_control_node)
    motion_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
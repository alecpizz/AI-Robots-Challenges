"""spot_controller3 controller."""
import copy

import numpy as np
import sys

from Bezier import BezierGait
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, keyboard
from controller import Supervisor
from SpotKinematics import SpotModel
import time
from pynput.keyboard import Key, Listener

moveBindings = {
    'i': (0.5, 0, 0, 0),
    'j': (0.1, 0, 0, 1),
    'l': (0.1, 0, 0, -1),
    ',': (-1, 0, 0, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

rollBindings = {
    'a': 0.1,
    's': -0.1,
}

pitchBindings = {
    'd': 0.1,
    'f': -0.1,
}
yawBindings = {
    'g': 0.1,
    'h': -0.1,
}

xyzBindings = {
    '1': (0.01, 0, 0),
    '2': (-0.01, 0, 0),
    '3': (0, 0.01, 0),
    '4': (0, -0.01, 0),
    '5': (0, 0, 0.01),
    '6': (0, 0, -0.01),
}


class SpotController():

    # create the Robot instance.
    for i in range(1, len(sys.argv)):
        print("argv[%i]=%s" % (i, sys.argv[i]))
    robot = Robot()
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    motor_names = [
                "front left shoulder abduction motor",
                "front left shoulder rotation motor",
                "front left elbow motor",
                "front right shoulder abduction motor",
                "front right shoulder rotation motor",
                "front right elbow motor",
                "rear left shoulder abduction motor",
                "rear left shoulder rotation motor",
                "rear left elbow motor",
                "rear right shoulder abduction motor",
                "rear right shoulder rotation motor",
                "rear right elbow motor",
            ]

    motors = []

    for motor_name in motor_names:
        motors.append(robot.getDevice(motor_name))

    touch_fl = robot.getDevice("front left touch sensor")
    touch_fr = robot.getDevice("front right touch sensor")
    touch_rl = robot.getDevice("rear left touch sensor")
    touch_rr = robot.getDevice("rear right touch sensor")

    touch_fl.enable(timestep)
    touch_fr.enable(timestep)
    touch_rl.enable(timestep)
    touch_rr.enable(timestep)

    spot = SpotModel()
    T_bf0 = spot.WorldToFoot
    T_bf = copy.deepcopy(T_bf0)
    bzg = BezierGait(dt=0.032)

    #bezier gait control inputs
    xd = float(sys.argv[1])
    yd = float(sys.argv[2])
    zd = float(sys.argv[3])
    rolld = 0.0
    pitchd = 0.0
    yawd = 0.0
    StepLength = 0.0
    LateralFraction = 0.0
    YawRate = 0.0
    StepVelocity = 0.0
    ClearanceHeight = 0.0
    PenetrationDepth = 0.0
    SwingPeriod = 0.0
    YawControl = 0.0
    YawControlOn = False
    #spot states
    x_inst = 0.0
    y_inst = 0.0
    z_inst = 0.0
    roll_inst = 0.0
    pitch_inst = 0.0
    yaw_inst = 0.0
    search_index = -1

    front_left_lower_leg_contact = 1
    front_right_lower_leg_contact = 1
    rear_left_lower_leg_contact = 1
    rear_right_lower_leg_contact = 1


    n_steps_to_achieve_target = 0
    step_difference = [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
    m_target = []
    # You should insert a getDevice-like function in order to get the
    # instance of a device of the robot. Something like:
    #  motor = robot.getDevice('motorname')
    #  ds = robot.getDevice('dsname')
    #  ds.enable(timestep)

    def yaw_control(self):
        """Yaw body controller"""
        yaw_target = self.YawControl
        thr = np.pi / 2
        if (yaw_target > thr and self.yaw_inst < -thr) or (
                self.yaw_inst > thr and yaw_target < -thr
        ):
            residual = (yaw_target - self.yaw_inst) * np.sign(
                yaw_target - self.yaw_inst
            ) - 2 * np.pi
            yawrate_d = 2.0 * np.sqrt(abs(residual)) * np.sign(residual)
        else:
            residual = yaw_target - self.yaw_inst
            yawrate_d = 4.0 * np.sqrt(abs(residual)) * np.sign(residual)
        return yawrate_d

    def apply_motor(self, motors_target_pos):
        motor_offsets = [0.0, 0.52, -1.182]
        for i in range(len(self.motors)):
            self.motors[i].setPosition(motor_offsets[i % 3] + motors_target_pos[i])
        # for idx, motor in enumerate(motors_target_pos):
        #     motor.setPosition(motor_offsets[idx % 3] + motors_target_pos[idx])

    def spot_inverse_control(self):
        pos = np.array([self.xd, self.yd, self.zd])
        orn = np.array([self.rolld, self.pitchd, self.yawd])
        # yaw controller
        if self.YawControlOn == 1.0:
            YawRate_desired = self.yaw_control()
        else:
            YawRate_desired = self.YawRate
        # Update Swing Period
        self.bzg.Tswing = self.SwingPeriod
        contacts = [
            self.front_left_lower_leg_contact,
            self.front_right_lower_leg_contact,
            self.rear_left_lower_leg_contact,
            self.rear_right_lower_leg_contact,
        ]
        # Get Desired Foot Poses
        self.T_bf = self.bzg.GenerateTrajectory(
            self.StepLength,
            self.LateralFraction,
            YawRate_desired,
            self.StepVelocity,
            self.T_bf0,
            self.T_bf,
            self.ClearanceHeight,
            self.PenetrationDepth,
            contacts,
        )
        joint_angles = -self.spot.IK(orn, pos, self.T_bf)
        target = [
            joint_angles[0][0],
            joint_angles[0][1],
            joint_angles[0][2],
            joint_angles[1][0],
            joint_angles[1][1],
            joint_angles[1][2],
            joint_angles[2][0],
            joint_angles[2][1],
            joint_angles[2][2],
            joint_angles[3][0],
            joint_angles[3][1],
            joint_angles[3][2],
        ]
        self.apply_motor(target)

    def step(self):
        return self.robot.step(self.timestep)

    def timer_ended(self):
        speed = 1.0
        turn = 0.5

def on_press(key):
    keych = key.char
    if keych in moveBindings.keys():
        control.xd = moveBindings[keych][0]
        pass
    elif keych in speedBindings.keys():
        pass
    elif keych in rollBindings.keys():
        pass
    elif keych in pitchBindings.keys():
        pass
    elif keych in yawBindings.keys():
        pass
    elif keych in xyzBindings.keys():
        pass
    else:
        pass
    pass

def on_release(key):
    pass

# Main loop:
control = SpotController()
current_time = time.time()
prev_time = time.time()
loopTimer = 0.0
loopTime = 0.1

listener = Listener(on_press=on_press, on_release=on_release)
listener.start()
# - perform simulation steps until Webots is stopping the controller
while control.step() != -1:

    control.front_left_lower_leg_contact = control.touch_fl.getValue()
    control.front_right_lower_leg_contact = control.touch_fr.getValue()
    control.rear_left_lower_leg_contact = control.touch_rl.getValue()
    control.rear_right_lower_leg_contact = control.touch_rr.getValue()

    control.StepLength = 0.5
    control.StepVelocity = 0.01
    control.spot_inverse_control()
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass
listener.stop()
# Enter here exit cleanup code.

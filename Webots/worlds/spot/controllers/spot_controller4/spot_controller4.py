import sys, tempfile
from controller import Robot
import ikpy
from ikpy.chain import Chain
import math
from controller import Supervisor

IKPY_MAX_ITERATIONS = 4

supervisor = Supervisor()
timestep = int(4 * supervisor.getBasicTimeStep())

filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(supervisor.getUrdf().encode('utf-8'))
armChain = Chain.from_urdf_file(filename)

motors = []
for link in armChain.links:
    print(link.name)
    if 'motor' in link.name:
        motor = supervisor.getDevice(link.name)
        motor.setVelocity(1.0)
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timestep)
        motors.append(motor)

target = supervisor.getFromDef('TARGET')
arm = supervisor.getSelf()

while supervisor.step(timestep) != -1:
    targetPosition = target.getPosition()
    armPosition = arm.getPosition()

    # Compute the position of the target relatively to the arm.
    # x and y axis are inverted because the arm is not aligned with the Webots global axes.
    x = -(targetPosition[1] - armPosition[1])
    y = targetPosition[0] - armPosition[0]
    z = targetPosition[2] - armPosition[2]

    # Call "ikpy" to compute the inverse kinematics of the arm.
    initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
    ikResults = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=initial_position)

    # Recalculate the inverse kinematics of the arm if necessary.
    position = armChain.forward_kinematics(ikResults)
    squared_distance = (position[0, 3] - x) ** 2 + (position[1, 3] - y) ** 2 + (position[2, 3] - z) ** 2
    if math.sqrt(squared_distance) > 0.03:
        ikResults = armChain.inverse_kinematics([x, y, z])

    # Actuate the arm motors with the IK results.
    for i in range(len(motors)):
        motors[i].setPosition(ikResults[i + 1])

# robot = Robot()

# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())
# fl_shoulder_abduction = robot.getDevice("front left shoulder abduction motor")
# fl_shoulder_rotation = robot.getDevice("front left shoulder rotation motor")
# fl_elbow = robot.getDevice("front left elbow motor")


# Main loop:
# - perform simulation steps until Webots is stopping the controller
# while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.

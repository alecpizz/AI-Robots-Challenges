from controller import Robot
from controller import Motor
import math

# High level functions
def move_forward(): 
    left_motor.setVelocity(max_speed)
    right_motor.setVelocity(max_speed)

def move_back():
    left_motor.setVelocity(-max_speed)
    right_motor.setVelocity(-max_speed)

def turn_left(): 
    left_motor.setVelocity(-max_speed)
    right_motor.setVelocity(max_speed)

def turn_right(): 
    left_motor.setVelocity(max_speed)
    right_motor.setVelocity(-max_speed)

def reset_motors():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)


#--------------------main function--------------------
if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    
    #Variables
    time_step = 64
    max_speed = .25
    magic_turn_num = 4.87
    turn_step_size =  math.floor(time_step * magic_turn_num * (1 + max_speed))
    print(turn_step_size)

    #Set up motor/track wheels
    left_motor = robot.getDevice("leftMotor")
    right_motor = robot.getDevice("rightMotor")
    left_motor.setPosition(math.inf)
    right_motor.setPosition(math.inf)

    reset_motors()
    
    #Robot AI goes here **************
    i = 0
    while i < 4:
        move_forward()
        robot.step(time_step * 8)
        turn_right()
        robot.step(turn_step_size)
        i = i + 1
    
    i = 0
    while i < 4:
        move_back()
        robot.step(time_step * 8)
        turn_right()
        robot.step( turn_step_size)
        i = i + 1
        

    # move_forward()
    # robot.step(time_step * 8)
    # turn_right()
    # robot.step( math.floor(time_step * magic_turn_num * max_speed))
    # move_forward()
    # robot.step(time_step * 8)
    # turn_right()
    # robot.step( math.floor(time_step * magic_turn_num * max_speed))
    # move_forward()
    # robot.step(time_step * 8)
    # turn_right()
    # robot.step( math.floor(time_step * magic_turn_num * max_speed))
    # move_forward()
    # robot.step(time_step * 8)
    # turn_right()
    # robot.step( math.floor(time_step * magic_turn_num * max_speed))
    
    
    #End of AI
    
    reset_motors()
    del robot
    pass  # EXIT_SUCCESS
    
    
    
    
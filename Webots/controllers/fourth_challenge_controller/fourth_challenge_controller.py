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

def drive_forward(angle, inRadians = False): 
    if inRadians: 
        angle *= 57.295   #//180/3.14 = 57
    if angle > 90 or angle < -90:
        pass
    elif angle >= 0:
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity( max_speed * math.sin((angle + 45) / 28.648) )
    else:
        left_motor.setVelocity(max_speed * math.sin((-angle + 45) / 28.648) )
        right_motor.setVelocity(max_speed)

def set_camera_position(rotation):
    camera_motor.setPosition(rotation)
    
def get_camera_position():
    return camera_motor.getTargetPosition()

def reset_motors():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    camera_motor.setVelocity(0)
    
    
#--------------------main function--------------------
if __name__ == "__main__":
    #Create the Robot instance.
    robot = Robot()
    
    #Variables
    time_step = 32
    max_speed = 0.65
    camera_min_position =  0.65 #lowest position
    camera_max_position = -1 #highest position
    camera_default_position = 0 #base position
    
    #Set up motor/track wheels
    left_motor = robot.getDevice("leftMotor")
    right_motor = robot.getDevice("rightMotor")
    left_motor.setPosition(math.inf)
    right_motor.setPosition(math.inf)
    
    #Set up camera and camera motor
    camera = robot.getDevice("CameraArm_camera")
    camera.enable(time_step)
    width = camera.getWidth()
    height = camera.getWidth()
    
    #Setting maximum rotational values of camera motor
    camera_motor = robot.getDevice("CameraArm_rotational_motor")
    camera_motor.maxPosition = camera_max_position
    camera_motor.minPosition = camera_min_position
    set_camera_position(camera_min_position)
    #Set random duck placement (change to 'True' or 'False')
    robot.custom_data = 'False'
    
    kp = 1.5
    kd = 1.5
    prev_error = 0
    
    def pid(error):
        global prev_error
        prop = error
        diff = error - prev_error
        correct = (kp * prop) + (kd * diff)
        prev_error = error
        return correct
       
    def setSpeed(base_speed, balance):
        print(balance)
        left_motor.setVelocity(-base_speed - balance)
        right_motor.setVelocity(base_speed + balance)
    
    #Main Loop while simulation is running
    while robot.step(time_step) != -1:
        #Code goes here
        
        #RGB Values for debugging center pixel
        red = camera.imageGetRed(camera.getImage(), int(width), int(width / 2), int(height / 2))
        green = camera.imageGetGreen(camera.getImage(), int(width), int(width / 2), int(height / 2))
        blue = camera.imageGetBlue(camera.getImage(), int(width), int(width / 2), int(height / 2))
        print("Red: " + str(red) + ", Green: " + str(green) + ", Blue: " + str(blue))
        # #green greater than 145, we're off the wall, turn left
        #green greater than 234 and red greeater than 234, we're in the white, turn right
        if green > 110 and blue > 90 and blue < 110:
            # turn left
            # error = 2
            # setSpeed(4, pid(error))
            drive_forward(-30)
        elif green > 200 and blue > 190 and red > 200:
            # turn_right
            # error = -2
            drive_forward(30)
        else:
            # drive forward
            # error = 0
            # setSpeed(4, pid(error))
            drive_forward(0)
        pass #end of code region
                
    #End of AI
    reset_motors()
    del robot
    pass  # EXIT_SUCCESS

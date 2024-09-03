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
        
def drive_backward(angle, inRadians = False):
    if inRadians: 
        angle *= 57.295   #//180/3.14 = 57
    if angle > 90 or angle < -90:
        pass
    elif angle >= 0:
        left_motor.setVelocity(-max_speed)
        right_motor.setVelocity( -max_speed * math.sin((angle + 45) / 28.648) )
    else:
        left_motor.setVelocity(-max_speed * math.sin((-angle + 45) / 28.648) )
        right_motor.setVelocity(-max_speed)

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
    time_step = 64
    max_speed = 0.5
    camera_min_position =  0.65 #lowest position
    camera_max_position = -1 #highest position
    camera_default_position = 0 #base position
    
    #Set up motor/track wheels
    left_motor = robot.getDevice("leftMotor")
    right_motor = robot.getDevice("rightMotor")
    left_motor.setPosition(math.inf)
    right_motor.setPosition(math.inf)
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
    #Set up camera and camera motor
    camera = robot.getDevice("CameraArm_camera")
    camera.enable(time_step)
    camera.recognitionEnable(time_step)
    width = camera.getWidth()
    height = camera.getWidth()
    
    #Setting maximum rotational values of camera motor
    camera_motor = robot.getDevice("CameraArm_rotational_motor")
    camera_motor.maxPosition = camera_max_position
    camera_motor.minPosition = camera_min_position
    set_camera_position(camera_min_position)

    #Set random duck placement (change to 'True' or 'False')
    robot.custom_data = 'False'
    
    drive_forward(-35)
    robot.step(time_step * 8)
    drive_forward(0)
    robot.step(time_step * 12)
    drive_forward(75)
    robot.step(time_step * 6)
    max_speed = .35
    move_forward()
    robot.step(int(time_step * 19))
    
    
    move_back()
    robot.step(int(time_step * 18))
    max_speed = 0.5 
    drive_backward(75)
    robot.step(time_step * 6)
    drive_backward(0)
    robot.step(time_step * 12)
    drive_forward(70)
    robot.step(time_step * 8)
    drive_forward(0)
    robot.step(time_step * 12)
    drive_forward(-70)
    robot.step(time_step * 5)
    turn_left()
    robot.step(2)
    drive_forward(0)
    robot.step(time_step * 12)
    turn_left()
    robot.step(2)
    drive_forward(0)
    robot.step(time_step * 3)
    turn_left()
    robot.step(time_step)
    drive_forward(0)
    robot.step(time_step)
    #Main Loop while simulation is running
    # while robot.step(time_step) != -1:
        #Code goes here
        
        #RGB Values for debugging center pixel
        # red = camera.imageGetRed(camera.getImage(), int(width), int(width / 2), int(height / 2))
        # green = camera.imageGetGreen(camera.getImage(), int(width), int(width / 2), int(height / 2))
        # blue = camera.imageGetBlue(camera.getImage(), int(width), int(width / 2), int(height / 2))
        # print("Red: " + str(red) + ", Green: " + str(green) + ", Blue: " + str(blue))
        # size = len(camera.getRecognitionObjects())
        # if(size <= 0):
            # drive_forward(0)
            # continue
        # for x in camera.getRecognitionObjects():
            # colors = x.getColors()
            # if colors[0] > .68 and colors[1] > 0.54 and colors[2] > 0.49:
                # print("look a wall")
                # break
            # elif colors[0] < 0.1 and colors[1] > 0.62 and colors[2] > 0.93:
                # print("look a ball")
                # drive_forward(-45)
                # break
            # else:
                # reset_motors()
                # break
    pass #end of code region
                
    #End of AI
    reset_motors()
    del robot
    pass  # EXIT_SUCCESS

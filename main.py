from time import sleep
from machine import I2C, Pin, PWM
from encoder import Encoder
from motor import Motor
import VariConPins as VCP
from ultrasonic import sonic
from ssd1306 import SSD1306_I2C
from APDS9960LITE import APDS9960LITE




# Initialise the motors and encoders
Left_Motor = Motor("left", VCP.LMP1, VCP.LMP2, VCP.LMP3)
Right_Motor = Motor("right", VCP.RMP1, VCP.RMP2, VCP.RMP3)
Encoder_Left = VCP.encPinL
Encoder_Right = VCP.encPinR
enc = Encoder(Encoder_Left, Encoder_Right)


# define ultrasonic
ECHO = VCP.ECHOPin
TRIG = VCP.TRIGPin
ultrasonic_sensor = sonic(TRIG, ECHO)

# define servo object
pwm_servo = PWM(Pin(VCP.servoPin))
pwm_servo.freq(VCP.servoPWM)


# define IR sensors and LED objects
IR_sensorL = Pin(VCP.IRLeftPin, Pin.IN)
IR_sensorC = Pin(VCP.IRCentrePin, Pin.IN)
IR_sensorR = Pin(VCP.IRRightPin, Pin.IN)

LED_green = Pin("LED", Pin.OUT)
VCP.ir_sens_readL = IR_sensorL.value()
VCP.ir_sens_readC = IR_sensorC.value()
VCP.ir_sens_readR = IR_sensorR.value()

# Define OLED object
# i2c = I2C(0, sda=Pin(VCP.oledSDA), scl=Pin(VCP.oledSCL))
# oled = SSD1306_I2C(128, 64, i2c)
#
# oled.text("IRL={}".format(VCP.ir_sens_readL), 0, 0)
# oled.text("IRC={}".format(VCP.ir_sens_readC), 0, 20)
# oled.text("IRR={}".format(VCP.ir_sens_readR), 0, 40)
# oled.text("USL={}".format(VCP.US_distL), 56, 0)
# oled.text("USC={}".format(VCP.US_distF), 56, 20)
# oled.text("USR={}".format(VCP.US_distR), 56, 40)
# oled.show()  # Note: 0,0 top left, 128, 64 bottom right. 128 X 64 CORDs


# Define RBG sensor object
i2c = I2C(0, sda=Pin(VCP.rgbSDA), scl=Pin(VCP.rgbSCL))
apds9960 = APDS9960LITE(i2c)      # Create APDS9960 sensor object
apds9960.prox.enableSensor()    # Send I2C command to enable sensor


# ----------- ALL FUNCTIONS DEFINED HERE -------------------------------
# Sensors take initial values here
def sens_input():
    VCP.US_distF = ultrasonic_sensor.distance_mm()
    VCP.rgb_prox = apds9960.prox.proximityLevel
    for pos in range(0, 180, 15):
        setServoAngle(pos)  # Set servo to desired angle
        if pos == 15:
            VCP.US_distR = ultrasonic_sensor.distance_mm()
            sleep(0.2)
        if pos == 90:
            VCP.US_distF = ultrasonic_sensor.distance_mm()
            sleep(0.2)
        if pos == 165:
            VCP.US_distL = ultrasonic_sensor.distance_mm()
            sleep(0.2)
    setServoAngle(90)
    sleep(1)


# # Defines OLED init !!!!!! TODO Needs to be changed to print current state and state changes
# # def oled_init():
# #     oled.text("IRL={}".format(ir_sens_readL), 0, 0)
# #     oled.text("IRC={}".format(ir_sens_readC), 0, 20)
# #     oled.text("IRR={}".format(ir_sens_readR), 0, 40)
# #     oled.text("USL={}".format(VCP.US_distL), 56, 0)
# #     oled.text("USC={}".format(VCP.US_distF), 56, 20)
# #     oled.text("USR={}".format(VCP.US_distR), 56, 40)
# #     oled.show()  # Note: 0,0 top left, 128, 64 bottom right. 128 X 64 CORDs
#
# # !!! This section defines all movement functions
def move_forward():         # Used for any time the bot MOVES FORWARD ONLY
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()
    if VCP.encCountL <= 10:
        Left_Motor.duty(55)
    if VCP.encCountR <= 10:
        Right_Motor.duty(55)
    else:
        Left_Motor.duty(VCP.LeftLinearPWM)
        Right_Motor.duty(VCP.RightLinearPWM)

def move_backward():        # Used any time the bot MOVES BACKWARD ONLY
    Left_Motor.set_backwards()
    Right_Motor.set_backwards()
    Left_Motor.duty(VCP.LeftLinearPWM)
    Left_Motor.duty(VCP.RightLinearPWM)

def full_stop():
    Left_Motor.duty(0)
    Right_Motor.duty(0)

def sml_left_adj():            # TODO small left turn !!!! Needs testing and values
    Left_Motor.set_backwards()
    Right_Motor.set_forwards()
    while VCP.encAverage <= 5:
        Left_Motor.duty(50)
        Right_Motor.duty()

def sml_right_adj():            # TODO small right turn !!!! needs testing and values
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()
    Left_Motor.duty(50)
    Right_Motor.duty(0)

def static_pivot_l():              #TODO  pivot on the spot to left
    Left_Motor.set_backwards()
    Right_Motor.set_forwards()
    enc.clear_count()
    while VCP.encAverage <= 10:
        Left_Motor.duty(50)
        Right_Motor.duty(50)


def static_pivot_r():              # TODO pivot on the spot to right
    Left_Motor.set_forwards()
    Right_Motor.set_backwards()
    Left_Motor.duty()
    Right_Motor.duty()


# defines servo sweep function
def setServoAngle(angle):
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    range(0, 180, 15)
    pwm_servo.duty_u16(position)

def wall_follow():
    Left_Motor.duty(0)
    Right_Motor.duty(0)
    dist_F = ultrasonic_sensor.distance_mm()

    encLeft = 0
    encRight = 0

    pwm_servo.duty_u16(0)
    sleep(1)
    setServoAngle(90)
    sleep(0.5)
    pwm_servo.duty_u16(0)
    sleep(1)

    if dist_F < 100:
        setServoAngle(15)
        sleep(0.5)
        dist_R = ultrasonic_sensor.distance_mm()

        pwm_servo.duty_u16(0)
        sleep(2)
        setServoAngle(170)
        sleep(0.5)
        dist_L = ultrasonic_sensor.distance_mm()

        pwm_servo.duty_u16(0)
        sleep(2)
        setServoAngle(90)
        sleep(0.5)
        pwm_servo.duty_u16(0)

        if dist_R > dist_L:
            enc.clear_count()
            while enc.get_left() <= 10:
                Left_Motor.set_forwards()
                Left_Motor.duty(75)
                encLeft = enc.get_left()
                #print("enc left is {}".format(encLeft))
            Left_Motor.duty(0)
            sleep(1)
            while enc.get_right() <= 10:
                Right_Motor.set_forwards()
                Right_Motor.duty(75)
                encRight = enc.get_right()
                #print("enc right is {}".format(encRight))
        elif dist_L > dist_R:
            enc.clear_count()
            while enc.get_right() <= 10:
                Right_Motor.duty(75)
                encRight = enc.get_right()
                #print("enc right is {}".format(encRight))
            Right_Motor.duty(0)
            sleep(1)
            while enc.get_left() <= 10:
                Left_Motor.duty(75)
                encLeft = enc.get_left()
                #print("enc left is {}".format(encLeft))
        else:
            print("help")

def line_follow(): # TODO this needs to be developed

    while True:
        sens_input()

        if VCP.ir_sens_readC == 1:
            move_forward()

            if VCP.ir_sens_readL == 1:
                Left_Motor.set_forwards()
                Right_Motor.set_forwards()
                Left_Motor.duty(0)
                Right_Motor.duty(50)


            if VCP.ir_sens_readR == 1:
                Left_Motor.set_forwards()
                Right_Motor.set_forwards()
                Left_Motor.duty(50)
                Right_Motor.duty(0)

 # TODO Check this code out - test
def Welcome_Home():     # Runs after it has Returned to where it Believes the Garage is. Should play after line ends
    sens_input()
    encLeft = 0
    encRight = 0
    while VCP.US_distF < 200:   # Checks there is a wall in front of the goggles
        setServoAngle(15)
        sleep(0.5)

        pwm_servo.duty_u16(0)
        sleep(2)
        setServoAngle(165)
        sleep(0.5)

        pwm_servo.duty_u16(0)
        sleep(2)
        setServoAngle(90)
        sleep(0.5)
        pwm_servo.duty_u16(0)

    if VCP.US_distR < 200 and VCP.US_distL < 200:     # Checks there is a wall on either side of the bot, If true Garage Found

        if VCP.rgb_prox <= 0:     # Value must be finalised, Checking to see if it will hit the wall.
            move_forward()

        else:
            Left_Motor.duty(0)
            Right_Motor.duty(0)
        print('Welcome Home bitches')

    # Drives Forwards while it cant see a line or a wall # TODO

 # TODO Check this code out too - test
def Leaving_Home():     # Runs when the Bot starts
    dist_F = ultrasonic_sensor.distance_mm()    # Find Front Wall
    setServoAngle(90)
    encLeft = 0
    encRight = 0

    setServoAngle(15)   # Find Right Wall
    sleep(0.5)
    dist_R = ultrasonic_sensor.distance_mm()

    pwm_servo.duty_u16(0)   #Find left Wall
    sleep(2)
    setServoAngle(170)
    sleep(0.5)
    dist_L = ultrasonic_sensor.distance_mm()

    pwm_servo.duty_u16(0)   # go back to facing front
    sleep(2)
    setServoAngle(90)
    sleep(0.5)
    pwm_servo.duty_u16(0)

    ir_sens_readC = IR_sensorC.value()
    if dist_F and dist_L and dist_R < 250:  # Checks there is a wall in front of the goggles, If true, facing backwards
        setServoAngle(90)
        dist_F = ultrasonic_sensor.distance_mm()
        setServoAngle(15)  # Find Right Wall
        sleep(0.5)
        dist_R = ultrasonic_sensor.distance_mm()

        pwm_servo.duty_u16(0)  # Find left Wall
        sleep(2)
        setServoAngle(170)
        sleep(0.5)
        dist_L = ultrasonic_sensor.distance_mm()

        pwm_servo.duty_u16(0)  # go back to facing front
        sleep(2)
        setServoAngle(90)
        sleep(0.5)
        pwm_servo.duty_u16(0)
        if dist_F and dist_L and dist_R != 0:   # While it reads a value for the Garage
            Left_Motor.set_backwards()      # Books it backwards until it is out of the garage
            Right_Motor.set_backwards()

            # balanced PWM output using encoder
            encCountL = enc.get_left()
            encCountR = enc.get_right()
            pwmL = 45 + 3 * (encCountR - encCountL)
            pwmR = 45 + 3 * (encCountL - encCountR)
            Left_Motor.duty(pwmL)
            Right_Motor.duty(pwmR)
            dist_F = ultrasonic_sensor.distance_mm()    # Find Front wall

            setServoAngle(15)  # Find Right Wall
            sleep(0.5)
            dist_R = ultrasonic_sensor.distance_mm()

            pwm_servo.duty_u16(0)  # Find left Wall
            sleep(2)
            setServoAngle(170)
            sleep(0.5)
            dist_L = ultrasonic_sensor.distance_mm()

            pwm_servo.duty_u16(0)  # go back to facing front
            sleep(2)
            setServoAngle(90)
            sleep(0.5)
            pwm_servo.duty_u16(0)

        else:
            static_pivot_l()

# variables that need retained value in loops
dir_left = 0
dir_right = 0
servo_position = 0
goal_count = 0
direction = 0
action = 0
thru_count = 0

# # ----------------------------- state machine code starts here ----------------------------- # #

# Declare states
state_list = {0: 'Stopped', 1: 'Driving', 2: 'Reversing', 3: 'Wall_Follow', 4: 'Line_Follow', 5: 'Hallway',
              6: 'Converging_Hallway', 7: 'Roundabout', 8: 'Start'}

# Default state when init is 'Start'
state = state_list[8]
print("Current state is '{}'".format(state))    # TODO Update to OLED


while thru_count <= 100:             # # ----- States and transition conditions ----- # #
    sens_input()
    print(VCP.ir_sens_readC, VCP.ir_sens_readR, VCP.ir_sens_readL)
    print(VCP.US_distF, VCP.US_distL, VCP.US_distR)

    if state == 'Start':       # state 8
        sens_input()
        dir_left = 'stop'
        dir_right = 'stop'
        sleep(2)                    # 2 seconds of nothing then
        state = state_list[1]       # changes state from Stopped to Driving
        print("State changed to '{}'!".format(state))   # TODO Update to OLED
        sens_input()

    if state == 'Stopped':     # state 0
        sens_input()
        print("Current state is '{}'".format(state))    # TODO Update to OLED
        dir_left = 'stop'
        dir_right = 'stop'
        sleep(2)

        if VCP.ir_sens_readC == 1:  # check ir sensors > if white detected
            state = state_list[4]  # switch to LINE_Follow
            print("State changed to '{}'!".format(state))  # TODO Update to OLED

        elif VCP.US_distR >= 250 or VCP.US_distL >= 250:  # check US right/left dist if >= 250 then reverse
            state = state_list[2]  # reverse
            print("State changed to '{}'!".format(state))  # TODO Update to OLED
        sens_input()



    if state == 'Driving':          # state 1
        sens_input()

        print(VCP.ir_sens_readC, VCP.ir_sens_readR, VCP.ir_sens_readL)
        print(VCP.US_distF, VCP.US_distL, VCP.US_distR)
        print("Current state is '{}'".format(state))    # TODO Update to OLED
        dir_left = 'fwd'
        dir_right = 'fwd'

        if VCP.ir_sens_readL or VCP.ir_sens_readC or VCP.ir_sens_readR:
            state = state_list[4]  # SWITCH TO LINE_FOLLOW
            print("State changed to '{}'!".format(state))  # TODO Update to OLED

        elif VCP.rgb_prox >= 5:
            state = state_list[0]  # STOPS on obstacle detection
            print("State changed to '{}'!".format(state))  # TODO Update to OLED

        elif 0 < VCP.US_distL <= 250 or 0 < VCP.US_distR <= 250:
            state = state_list[3]  # SWITCH TO WALL_FOLLOW
            print("State changed to '{}'!".format(state))  # TODO Update to OLED

        elif VCP.US_distL <= 250 and 0 < VCP.US_distR <= 250 and VCP.US_distF <=250:
            Welcome_Home()
        sens_input()


    if state == 'Reversing':           # state 2
        sens_input()
        print("Current state is '{}'".format(state))    # TODO Update to OLED
        dir_left = 'rev'
        dir_right = 'rev'
        if VCP.ir_sens_readL == 1 or VCP.ir_sens_readC == 1 or VCP.ir_sens_readR == 1:
            state = state_list[4]  # SWITCH TO LINE_FOLLOW
            print("State changed to '{}'!".format(state))  # TODO Update to OLED

        elif VCP.US_distL <= 300 or VCP.US_distR <= 300:
            state = state_list[3]  # SWITCH TO WALL_FOLLOW
            print("State changed to '{}'!".format(state))  # TODO Update to OLED

        elif VCP.ir_sens_readC == 0:
            if VCP.US_distF > 300:
                state = state_list[1] # SWITCH TO DRIVING
                print("State changed to '{}'!".format(state))  # TODO Update to OLED

        # this will initiate pivot function and turn 180.
        sleep(1)
        dir_left = 'rev'
        dir_right = 'fwd'
        sleep(1)
        state = state_list[1] # SWITCH TO DRIVING


    if state == 'Wall_Follow':          # state 3
        sens_input()
        print("Current state is '{}'".format(state))    # TODO Update to OLED
        Left_Motor.set_forwards()
        Right_Motor.set_forwards()

        print(VCP.ir_sens_readC, VCP.ir_sens_readR, VCP.ir_sens_readL)

        if VCP.ir_sens_readL or VCP.ir_sens_readC or VCP.ir_sens_readR:
            state = state_list[4]  # SWITCH TO LINE_FOLLOW
            print("State changed to '{}'!".format(state))  # TODO Update to OLED

        if VCP.ir_sens_readC == 0:
            if VCP.US_distL > 150 and VCP.US_distR > 150:
                state = state_list[1] # switch to drive
                print("State changed to '{}'!".format(state))  # TODO Update to OLED

        # if VCP.rgb_prox >= 5:
        #     state = state_list[0]  # STOPS on obstacle detection
        #     print("State changed to '{}'!".format(state))  # TODO Update to OLED
            else:
                action = 'wall_follow'
        sens_input()



    if state == 'Line_Follow':          # state 4   # TODO THIS NEEDS EXPANDING
        sens_input()
        print("Current state is '{}'".format(state))
        Left_Motor.set_forwards()
        Right_Motor.set_forwards()

        if VCP.ir_sens_readC == 0:
            if VCP.US_distL > 300 and VCP.US_distR > 300:
                state = state_list[1]  # switch to drive
                print("State changed to '{}'!".format(state))  # TODO Update to OLED
            else:
                action = 'line_follow'
        sens_input()


    if state == 'Roundabout':           # state 7       # TODO THIS NEEDS EXPANDING
        sens_input()
        print("Current state is '{}'".format(state))    # TODO Update to OLED
        goal_count += 1
        print("Goal {} of 4 found!".format(goal_count))     # TODO Update to OLED
        sens_input()


# # ------------------------------- Start of control logic ------------------------------- # #

    if dir_left == 'fwd' and dir_right == 'fwd':
        move_forward()

    if dir_left == 'rev' and dir_right == 'rev':
        enc.clear_count()
        while VCP.encAverage <= 20:   # Does this enc.Average work as expected??
            move_backward()

    if dir_left == 'stop' and dir_right == 'stop':
        full_stop()

    if dir_left == 'rev' and dir_right == 'fwd':
        enc.clear_count()
        static_pivot_l()
        sleep(1)

    if action == 'wall_follow':
        sens_input()
        wall_follow()

    if action == 'line_follow':
        sens_input()
        line_follow()

    thru_count = 0

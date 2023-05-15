from time import sleep

from machine import I2C, Pin, PWM

import VariConPins as VCP
from encoder import Encoder
from motor import Motor
from ssd1306 import SSD1306_I2C
from ultrasonic import sonic

# Initialise the motors and encoders
Left_Motor = Motor("left", VCP.LMP1, VCP.LMP2, VCP.LMP3)
Right_Motor = Motor("right", VCP.RMP1, VCP.RMP2, VCP.RMP3)
Encoder_Left = VCP.encPinL
Encoder_Right = VCP.encPinR
enc = Encoder(Encoder_Left, Encoder_Right)

# define ultrasonic 1
ECHO_1 = VCP.ECHOPin_1
TRIG_1 = VCP.TRIGPin_1
ultrasonic_sensor_1 = sonic(TRIG_1, ECHO_1)

# define ultrasonic 2
ECHO_2 = VCP.ECHOPin_2
TRIG_2 = VCP.TRIGPin_2
ultrasonic_sensor_2 = sonic(TRIG_2, ECHO_2)

# define servo object
pwm_servo = PWM(Pin(VCP.servoPin))
pwm_servo.freq(VCP.servoPWM)

# define IR sensors and LED objects
IR_sensorL = Pin(VCP.IRLeftPin, Pin.IN)
IR_sensorC = Pin(VCP.IRCentrePin, Pin.IN)
IR_sensorR = Pin(VCP.IRRightPin, Pin.IN)

LED_green = Pin("LED", Pin.OUT)


# Define OLED object
i2c = I2C(0, sda=Pin(VCP.oledSDA), scl=Pin(VCP.oledSCL))
oled = SSD1306_I2C(128, 64, i2c)

# sensor variables defined here >>> updated throughout code
US_distF = 0
US_distL = 0
US_distR = 0
ir_sens_readL = 0
ir_sens_readC = 0
ir_sens_readR = 0


# ----------- ALL FUNCTIONS DEFINED HERE -------------------------------

# Sensors take initial values here
def sens_input():
    US_distL = ultrasonic_sensor_1.distance_mm()
    US_distF = ultrasonic_sensor_2.distance_mm()
    US_distR = ultrasonic_sensor_1.distance_mm()
    for pos in range(0, 180, 15):
        setServoAngle(pos)  # Set servo to desired angle
        if pos == 15:
            US_distR = ultrasonic_sensor_1.distance_mm()
            sleep(0.5)
        if pos == 165:
            US_distL = ultrasonic_sensor_1.distance_mm()
            sleep(0.5)
    for post in range(180, 0, 15):
        if pos == 165:
            US_distL = ultrasonic_sensor_1.distance_mm()
            sleep(0.5)
        if pos == 15:
            US_distR = ultrasonic_sensor_1.distance_mm()
            sleep(0.5)
    sleep(1)


# # Defines OLED update
def oled_update():
    oled.text("Goal Count = {}".format(goal_count), 0, 0)
    oled.show()


# !!! This section defines all movement functions
def move_forward():  # Used for any time the bot MOVES FORWARD ONLY
    enc.clear_count()
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()

    if VCP.encCountL <= 20:
        Left_Motor.duty(50)
    if VCP.encCountR <= 20:
        Right_Motor.duty(50)
    else:
        Left_Motor.duty(VCP.LeftLinearPWM)
        Right_Motor.duty(VCP.RightLinearPWM)


def move_backward():  # Used any time the bot MOVES BACKWARD ONLY
    Left_Motor.set_backwards()
    Right_Motor.set_backwards()
    Left_Motor.duty(VCP.LeftLinearPWM)
    Left_Motor.duty(VCP.RightLinearPWM)
    sleep(1)


def full_stop():
    Left_Motor.duty(0)
    Right_Motor.duty(0)


def static_pivot_l():  # TODO  pivot on the spot to left
    Left_Motor.set_forwards()
    Right_Motor.set_backwards()
    Left_Motor.duty(50)
    Right_Motor.duty(50)
    sleep(0.9)
    Right_Motor.duty(0)
    Left_Motor.duty(0)


# defines servo function
def setServoAngle(angle):
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    range(0, 180, 15)
    pwm_servo.duty_u16(position)


def wall_follow():
    Left_Motor.duty(0)
    Right_Motor.duty(0)
    US_distF = ultrasonic_sensor_1.distance_mm()

    encLeft = 0
    encRight = 0

    pwm_servo.duty_u16(0)
    sleep(1)
    setServoAngle(90)
    sleep(0.5)
    pwm_servo.duty_u16(0)
    sleep(1)

    if US_distF < 100:
        setServoAngle(15)
        sleep(0.5)
        dist_R = ultrasonic_sensor_1.distance_mm()

        pwm_servo.duty_u16(0)
        sleep(2)
        setServoAngle(170)
        sleep(0.5)
        dist_L = ultrasonic_sensor_1.distance_mm()

        pwm_servo.duty_u16(0)
        sleep(2)
        setServoAngle(90)
        sleep(0.5)
        pwm_servo.duty_u16(0)

        if US_distR > US_distL:
            enc.clear_count()
            while enc.get_left() <= 10:
                Left_Motor.set_forwards()
                Left_Motor.duty(75)
                encLeft = enc.get_left()
                # print("enc left is {}".format(encLeft))
            Left_Motor.duty(0)
            sleep(1)
            while enc.get_right() <= 10:
                Right_Motor.set_forwards()
                Right_Motor.duty(75)
                encRight = enc.get_right()
                # print("enc right is {}".format(encRight))
        elif US_distL > US_distR:
            enc.clear_count()
            while enc.get_right() <= 10:
                Right_Motor.duty(75)
                encRight = enc.get_right()
                # print("enc right is {}".format(encRight))
            Right_Motor.duty(0)
            sleep(1)
            while enc.get_left() <= 10:
                Left_Motor.duty(75)
                encLeft = enc.get_left()
                # print("enc left is {}".format(encLeft))
        else:
            print("help")


def line_follow():
    ir_sens_readC = IR_sensorC.value()
    ir_sens_readR = IR_sensorR.value()
    ir_sens_readL = IR_sensorL.value()

    # boolean operators to increase sensor speed
    centre_line = ir_sens_readC == 1
    left_line = ir_sens_readL == 1
    right_line = ir_sens_readR == 1

    while centre_line == 1 and not left_line and not right_line:
        move_forward()
        ir_sens_readC = IR_sensorC.value()
        centre_line = ir_sens_readC == 1

    while right_line and not centre_line:
        Right_Motor.duty(40)
        Left_Motor.duty(0)
        ir_sens_readR = IR_sensorR.value()
        right_line = ir_sens_readR == 1
        sleep(0.05)

    while left_line and not centre_line:
        Left_Motor.duty(50)
        Right_Motor.duty(0)
        ir_sens_readL = IR_sensorL.value()
        left_line = ir_sens_readL == 1
        sleep(0.05)

        # Smoother turning
        if left_line:
            Left_Motor.duty(25)
            Right_Motor.duty(50)
            sleep(0.1)
        elif right_line:
            Left_Motor.duty(50)
            Right_Motor.duty(25)
            sleep(0.1)

        while centre_line and right_line and left_line:
            Left_Motor.duty(0)
            Right_Motor.duty(0)

        return


def Leaving_Home():  # Runs when the Bot starts AFTER coming back home.

    move_backward()
    sleep(1)
    static_pivot_l()
    return


def ComingHome():
    ir_sens_readC = IR_sensorC.value()
    ir_sens_readR = IR_sensorR.value()
    ir_sens_readL = IR_sensorL.value()

    # boolean operators to increase sensor speed
    centre_line = ir_sens_readC == 1
    left_line = ir_sens_readL == 1
    right_line = ir_sens_readR == 1

    while centre_line == 1 and not left_line and not right_line:
        move_forward()
        ir_sens_readC = IR_sensorC.value()
        centre_line = ir_sens_readC == 1

    while right_line and not centre_line:
        Right_Motor.duty(40)
        Left_Motor.duty(0)
        ir_sens_readR = IR_sensorR.value()
        right_line = ir_sens_readR == 1
        sleep(0.05)

    while left_line and not centre_line:
        Left_Motor.duty(50)
        Right_Motor.duty(0)
        ir_sens_readL = IR_sensorL.value()
        left_line = ir_sens_readL == 1
        sleep(0.05)

        # Smoother turning
        if left_line:
            Left_Motor.duty(25)
            Right_Motor.duty(50)
            sleep(0.1)
        elif right_line:
            Left_Motor.duty(50)
            Right_Motor.duty(25)
            sleep(0.1)

        while centre_line and right_line and left_line:
            Left_Motor.duty(0)
            Right_Motor.duty(0)
        return

def Welcome_Home():  # Runs after it has Returned to where it Believes the Garage is. Should play after line ends

    dist_F = ultrasonic_sensor_1.distance_mm()
    while dist_F < 350:  # Checks there is a wall in front of the goggles
        setServoAngle(15)
        sleep(0.5)
        dist_R = ultrasonic_sensor_1.distance_mm()

        setServoAngle(90)
        sleep(0.5)
        dist_F = ultrasonic_sensor_1.distance_mm()

        pwm_servo.duty_u16(0)
        sleep(2)
        setServoAngle(170)
        sleep(0.5)
        dist_L = ultrasonic_sensor_1.distance_mm()

        pwm_servo.duty_u16(0)
        sleep(2)
        sleep(0.5)
        pwm_servo.duty_u16(0)
        while dist_R < 350 and dist_L < 350:  # Checks there is a wall on either side of the bot, If true Garage Found
            move_forward()
            dist_F = ultrasonic_sensor_1.distance_mm()
            if dist_F < 150:
                Left_Motor.duty(0)
                Right_Motor.duty(0)
                return
    move_forward()


def Roundabout(Rcount):  # count the amount of roundabouts that it has seen, and respond accordingly
    # will get us to the first roundabout and back
    static_pivot_l()
    ComingHome()
    return


def IR_update():
    ir_sens_readC = IR_sensorC.value()
    ir_sens_readR = IR_sensorR.value()
    ir_sens_readL = IR_sensorL.value()
    return ir_sens_readL, ir_sens_readC, ir_sens_readR

# variables that need retained value in loops
dir_left = 0
dir_right = 0
servo_position = 0
Rcount = 0
goal_count = 0
direction = 0
action = 0

# # ----------------------------- state machine code starts here ----------------------------- # #

# Declare states
# state_list = {0: 'Stopped', 1: 'Driving', 2: 'Reversing', 3: 'Wall_Follow', 4: 'Line_Follow', 5: 'Hallway',
#               6: 'Converging_Hallway', 7: 'Roundabout', 8: 'Start', 9: 'Garage'}
state_list = {0: 'Start', 1: 'One found', 2: 'Two found', 3: 'Three found', 4: 'Four found'}
# Default state when init is 'Start'
state = state_list[0]
oled_update()
print("Current state is '{}'".format(state))

while True:  # # ----- States and transition conditions ----- # #

    ir_sens_readC, ir_sens_readR, ir_sens_readL = IR_update()

    if goal_count == 0 and state == 'Start':  # GOES TO FIRST ROUNDABOUT AND HEADS HOME
        while ir_sens_readC == 0 and ir_sens_readL == 0 and ir_sens_readR == 0:
            move_forward()
            ir_sens_readC, ir_sens_readR, ir_sens_readL = IR_update()

        if ir_sens_readC or ir_sens_readL or ir_sens_readR:
            line_follow()
            ir_sens_readC, ir_sens_readR, ir_sens_readL = IR_update()

        if ir_sens_readC and ir_sens_readL and ir_sens_readR:
            Left_Motor.duty(0)
            Right_Motor.duty(0)
            goal_count += 1
            oled_update()
            static_pivot_l()
            ir_sens_readC, ir_sens_readR, ir_sens_readL = IR_update()
            sleep(1)
            ComingHome()
            Welcome_Home()
            print("Garage reached!")
            sleep(5)
            state = state_list[1]
            sleep(2)

    # elif goal_count == 1 and state == 'One found':
    #     ir_sens_readC, ir_sens_readR, ir_sens_readL = IR_update()
    #     Leaving_Home()
    #
    #     while ir_sens_readC == 0 and ir_sens_readL == 0 and ir_sens_readR == 0:
    #         move_forward()
    #         ir_sens_readC, ir_sens_readR, ir_sens_readL = IR_update()
    #
    #     if ir_sens_readC or ir_sens_readL or ir_sens_readR:
    #         line_follow()
    #
    #     if ir_sens_readC and ir_sens_readL and ir_sens_readR:
    #         ir_sens_readC, ir_sens_readR, ir_sens_readL = IR_update()
    #         move_forward()
    #         sleep(1)
    #         line_follow()






    # if state == 'Start':  # state 8
    #     sens_input()
    #     dir_left = 'stop'
    #     dir_right = 'stop'
    #     sleep(2)  # 2 seconds of nothing then
    #     state = state_list[1]  # changes state from Stopped to Driving
    #     oled_update()
    #     print("State changed to '{}'!".format(state))
    #     sens_input()

    # if state == 'Stopped':  # state 0
    #     sens_input()
    #     print("Current state is '{}'".format(state))
    #     dir_left = 'stop'
    #     dir_right = 'stop'
    #     sleep(2)
    #     state = state_list[2]
    #     # if ir_sens_readL or ir_sens_readC or ir_sens_readR:  # check ir sensors > if white detected
    #     #     state = state_list[4]  # switch to LINE_Follow
    #     #     oled_update()
    #     #     print("State changed to '{}'!".format(state))
    #     #
    #     #
    #     # elif VCP.US_distR >= 250 or VCP.US_distL >= 250:  # check US right/left dist if >= 250 then reverse
    #     #     state = state_list[2]  # reverse
    #     #     oled_update()
    #     #     print("State changed to '{}'!".format(state))
    #     # sens_input()
    #
    # if state == 'Driving':  # state 1
    #
    #     dir_left = 'fwd'
    #     dir_right = 'fwd'
    #     #
    #     # if ir_sens_readL or ir_sens_readC or ir_sens_readR:
    #     #     state = state_list[4]  # SWITCH TO LINE_FOLLOW
    #     #     oled_update()
    #     #     print("State changed to '{}'!".format(state))
    #     #
    #     # elif US_distF <= 200:
    #     #     state = state_list[0]  # SWITCH TO STOPPED
    #     #     oled_update()
    #     #     print("State changed to '{}'!".format(state))
    #     #
    #     # elif 0 < US_distL <= 250 or 0 < US_distR <= 250:
    #     #     state = state_list[3]  # SWITCH TO WALL_FOLLOW
    #     #     oled_update()
    #     #     print("State changed to '{}'!".format(state))
    #     #
    #     # elif US_distL <= 250 and 0 < US_distR <= 250 and US_distF <= 250:
    #     #     state = state_list[9]
    #     #     oled_update()
    #     #     action = 'Home'
    #     sens_input()
    #
    # if state == 'Reversing':  # state 2
    #     sens_input()
    #     print("Current state is '{}'".format(state))
    #     dir_left = 'rev'
    #     dir_right = 'rev'
    #     if ir_sens_readL == 1 or ir_sens_readC == 1 or ir_sens_readR == 1:
    #         state = state_list[4]  # SWITCH TO LINE_FOLLOW
    #         oled_update()
    #         print("State changed to '{}'!".format(state))
    #
    #     elif US_distL <= 300 or US_distR <= 300:
    #         state = state_list[3]  # SWITCH TO WALL_FOLLOW
    #         oled_update()
    #         print("State changed to '{}'!".format(state))
    #
    #     elif ir_sens_readL == 0 and ir_sens_readC == 0 and ir_sens_readR == 0:
    #         if US_distF > 300:
    #             state = state_list[1]  # SWITCH TO DRIVING
    #             oled_update()
    #             print("State changed to '{}'!".format(state))
    #
    #     # this will initiate pivot function and turn 180.
    #     sleep(1)
    #     dir_left = 'rev'
    #     dir_right = 'fwd'
    #     sleep(1)
    #     state = state_list[1]  # SWITCH TO DRIVING
    #     oled_update()
    #
    # if state == 'Wall_Follow':  # state 3
    #     sens_input()
    #     print("Current state is '{}'".format(state))
    #     Left_Motor.set_forwards()
    #     Right_Motor.set_forwards()
    #
    #     if ir_sens_readL or ir_sens_readC or ir_sens_readR:
    #         state = state_list[4]  # SWITCH TO LINE_FOLLOW
    #         oled_update()
    #         print("State changed to '{}'!".format(state))
    #
    #     elif US_distF <= 200:
    #         state = state_list[0]
    #         oled_update()
    #         print("State changed to '{}'!".format(state))
    #
    #     if ir_sens_readL == 0 and ir_sens_readC == 0 and ir_sens_readR == 0:
    #         if US_distL > 200 and US_distR > 200:
    #             state = state_list[1]  # switch to drive
    #             oled_update()
    #             print("State changed to '{}'!".format(state))
    #
    #         else:
    #             action = 'wall_follow'
    #     sens_input()
    #
    # if state == 'Line_Follow':  # state 4   # TODO THIS NEEDS EXPANDING
    #     sens_input()
    #     print("Current state is '{}'".format(state))
    #     Left_Motor.set_forwards()
    #     Right_Motor.set_forwards()
    #
    #     if ir_sens_readL == 0 and ir_sens_readC == 0 and ir_sens_readR == 0:
    #         if US_distL > 300 and US_distR > 300:
    #             state = state_list[1]  # switch to drive
    #             oled_update()
    #             print("State changed to '{}'!".format(state))  # TODO Update to OLED
    #         else:
    #             action = 'line_follow'
    #
    #     elif US_distF <= 200:
    #         state = state_list[0]
    #         oled_update()
    #         print("State changed to '{}'!".format(state))
    #     sens_input()
    #
    # if state == 'Roundabout':  # state 7       # TODO THIS NEEDS EXPANDING
    #     sens_input()
    #     print("Current state is '{}'".format(state))  # TODO Update to OLED
    #     goal_count += 1
    #     print("Goal {} of 4 found!".format(goal_count))  # TODO Update to OLED
    #     sens_input()

    # # ------------------------------- Start of control logic ------------------------------- # #

    # if goal_count == 0 and state == 'Start':
    #     Leaving_Home()
    #     line_follow()
    #     if ir_sens_readC and ir_sens_readL and ir_sens_readR:
    #         ir_sens_readC = IR_sensorC.value()
    #         ir_sens_readR = IR_sensorR.value()
    #         ir_sens_readL = IR_sensorL.value()
    #         Left_Motor.duty(0)
    #         Right_Motor.duty(0)
    #         Rcount += 1
    #         Roundabout(Rcount)
    #         goal_count += 1
    #         sleep(3)
    #         move_backward()
    #         sleep(2)
    #         static_pivot_l()
    #         state = state_list[1]
    #
    #
    #
    # if dir_left == 'fwd' and dir_right == 'fwd':
    #     sens_input()
    #     Left_Motor.set_forwards()
    #     Right_Motor.set_forwards()
    #     Left_Motor.duty(50)
    #     Right_Motor.duty(50)
    #
    # if dir_left == 'rev' and dir_right == 'rev':
    #     enc.clear_count()
    #     while VCP.encAverage <= 20:  # Does this enc.Average work as expected??
    #         move_backward()
    #
    # if dir_left == 'stop' and dir_right == 'stop':  # and state != '':
    #     full_stop()
    #
    # if dir_left == 'rev' and dir_right == 'fwd':
    #     enc.clear_count()
    #     static_pivot_l()
    #     sleep(1)
    #
    # if action == 'wall_follow':
    #     sens_input()
    #     wall_follow()
    #
    # if action == 'line_follow':
    #     sens_input()
    #     line_follow()
    #
    # if action == 'Home':
    #     Welcome_Home()

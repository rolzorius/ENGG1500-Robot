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
ir_sens_readL = IR_sensorL.value()
ir_sens_readC = IR_sensorC.value()
ir_sens_readR = IR_sensorR.value()

# Define OLED object
i2c_0 = I2C(0, sda=Pin(VCP.oledSDA, VCP.rgbSDA), scl=Pin(VCP.oledSCL, VCP.rgbSCL))
oled = SSD1306_I2C(128, 64, i2c_0)

# Define RBG sensor object
i2c_1 = I2C(0, sda=Pin(VCP.rgbSDA), scl=Pin(VCP.rgbSCL))
apds9960 = APDS9960LITE(i2c_1)      # Create APDS9960 sensor object
apds9960.prox.enableSensor()    # Send I2C command to enable sensor


# ----------- ALL FUNCTIONS DEFINED HERE -------------------------------
# Sensors take initial values here
def sens_input():
    VCP.US_distF = ultrasonic_sensor.distance_mm()
    VCP.ir_sens_readL = IR_sensorL.value()
    VCP.ir_sens_readC = IR_sensorC.value()
    VCP.ir_sens_readR = IR_sensorR.value()
    VCP.rgb_prox = apds9960.prox.proximityLevel

# Defines OLED init !!!!!! Needs to be changed to print current state and state changes
def oled_init():
    oled.text("IRL={}".format(ir_sens_readL), 0, 0)
    oled.text("IRC={}".format(ir_sens_readC), 0, 20)
    oled.text("IRR={}".format(ir_sens_readR), 0, 40)
    oled.text("USL={}".format(VCP.US_distL), 56, 0)
    oled.text("USC={}".format(VCP.US_distF), 56, 20)
    oled.text("USR={}".format(VCP.US_distR), 56, 40)
    oled.show()  # Note: 0,0 top left, 128, 64 bottom right. 128 X 64 CORDs

# !!! This section defines all movement functions
def move_forward():         # Used for any time the bot MOVES FORWARD ONLY
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()
    Left_Motor.duty(VCP.LeftLinearPWM)
    Right_Motor.duty(VCP.RightLinearPWM)

def move_backward():        # Used any time the bot MOVES BACKWARD ONLY
    Left_Motor.set_backwards()
    Right_Motor.set_backwards()
    Left_Motor.duty(VCP.LeftLinearPWM)
    Left_Motor.duty(VCP.RightLinearPWM)

def sml_left_adj():            # small left turn !!!! Needs testing and values
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()
    Left_Motor.duty()
    Right_Motor.duty()

def sml_right_adj():            # small right turn !!!! needs testing and values
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()
    Left_Motor.duty(50)
    Right_Motor.duty(0)

def static_pivot_l():              # pivot on the spot to left
    Left_Motor.set_backwards()
    Right_Motor.set_forwards()
    Left_Motor.duty()
    Right_Motor.duty()
    sleep(2)

def static_pivot_r():              # pivot on the spot to right
    Left_Motor.set_forwards()
    Right_Motor.set_backwards()
    Left_Motor.duty()
    Right_Motor.duty()
    sleep(2)

# defines servo sweep function
def setServoAngle(angle):
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    range(0, 180, 30)
    pwm_servo.duty_u16(position)

def servo_sweep():
    while True:
        for pos in range(0, 180, 15):
            setServoAngle(pos)          # Set servo to desired angle
            if pos == 15:
                VCP.US_distR = ultrasonic_sensor.distance_mm()

            if pos == 90:
                VCP.US_distF = ultrasonic_sensor.distance_mm()

            if pos == 165:
                VCP.US_distL = ultrasonic_sensor.distance_mm()
        sleep(0.05)                # Wait 50 ms to reach angle
        for pos in range(180, 0, -15):
            setServoAngle(pos)          # Set servo to desired angle
            if pos == 15:
                VCP.US_distR = ultrasonic_sensor.distance_mm()

            if pos == 90:
                VCP.US_distF = ultrasonic_sensor.distance_mm()

            if pos == 165:
                VCP.US_distL = ultrasonic_sensor.distance_mm()
        sleep(0.05)


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
                print("enc left is {}".format(encLeft))
            Left_Motor.duty(0)
            sleep(1)
            while enc.get_right() <= 10:
                Right_Motor.set_forwards()
                Right_Motor.duty(75)
                encRight = enc.get_right()
                print("enc right is {}".format(encRight))
        elif dist_L > dist_R:
            enc.clear_count()
            while enc.get_right() <= 10:
                Right_Motor.duty(75)
                encRight = enc.get_right()
                print("enc right is {}".format(encRight))
            Right_Motor.duty(0)
            sleep(1)
            while enc.get_left() <= 10:
                Left_Motor.duty(75)
                encLeft = enc.get_left()
                print("enc left is {}".format(encLeft))
        else:
            print("help")


# variables that need retained value in loops
pwm_left = 0
pwm_right = 0
dir_left = 0
dir_right = 0
servo_position = 0
encCountL = 0
encCountR = 0
goal_count = 0

# Declare states
state_list = ['Stopped', 'Driving', 'Reversing', 'Wall_Follow', 'Line_Follow', 'Hallway',
              'Converging_Hallway', 'Roundabout', 'Garage']

# Default state when init is 'Stopped'
state = state_list[0]
print("Current state is '{}'".format(state)) # Update to OLED

while True:
    sens_input()                # Take initial sensor readings

    sleep(3)                    # 3 seconds of nothing then
    state = state_list[1]       # changes state from Stopped to Driving
    print("State changed to '{}'!".format(state))

    if state == 'Stopped':
        print("Current state is '{}'".format(state))
        Left_Motor.duty(0)
        Right_Motor.duty(0)

    if state == 'Driving':
        print("Current state is '{}'".format(state))    # Update to OLED
        dir_left = Left_Motor.set_forwards()
        dir_right = Right_Motor.set_forwards()

    if state == 'Reversing':
        print("Current state is '{}'".format(state))
        dir_left = Left_Motor.set_backwards()
        dir_right = Right_Motor.set_backwards()

    if state == 'Wall_Follow':
        print("Current state is '{}'".format(state))
        Left_Motor.set_forwards()
        Right_Motor.set_forwards()
        wall_follow()
        if VCP.US_distF > 250:
            state = state_list[1]
        if ir_sens_readC == 0:
            state = state_list[3]   # change state to Follow_Line
            print("State changed to '{}'!".format(state))


    if state == 'Line_Follow':      # THIS NEEDS EXPANDING
        print("Current state is '{}'".format(state))
        Left_Motor.set_forwards()
        Right_Motor.set_forwards()

    if state == 'Hallway':
        print("Current state is '{}'".format(state))
        # THIS NEEDS EXPANDING

    if state == 'Converging_Hallway':
        print("Current state is '{}'".format(state))
        # THIS NEEDS EXPANDING

    if state == 'Roundabout':
        print("Current state is '{}'".format(state))
        goal_count += 1
        print("Goal {} of 4 found!".format(goal_count))
        # THIS NEEDS EXPANDING



    # if (0 < servo_position < 90) and US_dist <= 250:
    #     state = state_list[2]
    #     print("State changed to '{}'!".format(state))
    #
    #             # move to follow_wall_R
    #     elif (90 < servo_position < 180) and US_dist <= 250:
    #         state = state_list[2]
    #         print("State changed to '{}'!".format(state))


# ------------------------------- Start of control logic
    while True:
        sens_input()
        if state == 'Driving':          # DRIVING FORWARDS
            move_forward()
            servo_sweep()
            if ir_sens_readL == 1:
                state = state_list[4]   # SWITCH TO LINE_FOLLOW
            if ir_sens_readC == 1:
                state = state_list[4]
            if ir_sens_readR == 1:
                state = state_list[4]
            if  VCP.rgb_prox >= 5:
                state = state_list[0]   # STOPS on obstacle detection

        if state == 'Reversing':      # DRIVING BACKWARDS
            move_backward()
            servo_sweep()
            if ir_sens_readL == 1:
                state = state_list[4]   # SWITCH TO LINE_FOLLOW
            if ir_sens_readC == 1:
                state = state_list[4]
            if ir_sens_readR == 1:
                state = state_list[4]

        elif state == 'Wall_Follow':
            sens_input()
            wall_follow()
            if VCP.rgb_prox >= 5:
                state = state_list[0]   # STOPS on obstacle detection







    # --------------------- obsolete at this point
    # # Control signals
    # while True:
    #     ir_sens_readL = IR_sensorL.value()
    #     ir_sens_readC = IR_sensorC.value()
    #     ir_sens_readR = IR_sensorR.value()
    #     func.setServoAngle()
    #     if dir_left == 'fwd' and dir_right == 'fwd':
    #         func.move_forward()
    #     elif dir_left == 'rev' and dir_right == 'rev':
    #         func.move_backward()


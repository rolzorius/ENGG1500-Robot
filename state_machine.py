from machine import Pin
from time import sleep
from motor import Motor
from encoder import Encoder
from ultrasonic import sonic
import VariConPins as VCP

# Initialise the motors and encoders
Left_Motor = Motor("left", 10, 11, 7)
Right_Motor = Motor("right", 9, 8, 6)
Encoder_Left = VCP.encPinL
Encoder_Right = VCP.encPinR
enc = Encoder(Encoder_Left, Encoder_Right)

def move_forward():         # Used for any time the bot MOVES FORWARD ONLY
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()
    Left_Motor.duty(VCP.LeftLinearPWM)
    Right_Motor.duty(VCP.RightLinearPWM)

def sml_L_adj():
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()
    Left_Motor.duty()
    Right_Motor.duty()

def sml_R_adj():            # small right turn
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()
    Left_Motor.duty()
    Right_Motor.duty()


# define ultrasonic
ECHO = VCP.ECHOPin
TRIG = VCP.TRIGPin
ultrasonic_sensor = sonic(TRIG, ECHO)

# define servo object
pwm_servo = PWM(Pin(VCP.servoPin))
pwm_servo.freq(VCP.servoPWM)

def setServoAngle(angle):
    position = int(VCP.angleConv)  # Convert angle into [1000, 9000]
    range(0, 180, 30)
    pwm_servo.duty_u16(position)

# define IR sensors and LED objects
IR_sensorL = Pin(VCP.IRLeftPin, Pin.IN)
IR_sensorC = Pin(VCP.IRCentrePin, Pin.IN)
IR_sensorR = Pin(VCP.IRRightPin, Pin.IN)

LED_green = Pin("LED", Pin.OUT)
ir_sens_readC = IR_sensorC.value()
ir_sens_readL = IR_sensorL.value()
ir_sens_readR = IR_sensorR.value()

# variables that need retained value in loops
pwm_left = 0
pwm_right = 0
dir_left = 0
dir_right = 0
servo_position = 0
encCountL = 0
encCountR = 0

# Declare states
state_list = ['Driving', 'Follow_Wall_L', 'Follow_Wall_R', 'Follow_Line', 'Hallway',
              'Converging_Hallway', 'Roundabout', 'Stopped']

# Default state when init is 'Stopped'
state = state_list[7]
print("Current state is '{}'".format(state)) # Update to OLED

while True:
    US_dist = ultrasonic_sensor.distance_mm()
    ir_sens_readC = IR_sensorC.value()
    ir_sens_readL = IR_sensorL.value()
    ir_sens_readR = IR_sensorR.value()
    sleep(3)                    # 3 seconds of nothing then
    state = state_list[0]       # changes state from Stopped to Driving

                # Driving state
    if state == 'Driving':
        print("Current state is '{}'".format(state))    # Update to OLED
        # dir_left = fwd
        # dir_right = fwd
        # pwm_left = LeftLinearPWM        # from ConVariPins.py
        # pwm_right = RightLinearPWM      # from ConVariPins.py
        move_forward()

        if ir_sens_readC == 0:
            state = state_list[3]
            print("Changed states to'{}'!".format(state_list))
            break

                # move to follow_wall_L
    if (0 < servo_position < 90) and US_dist <= 250:
        state = state_list[1]
        print("Current state is '{}".format(state))

                # move to follow_wall_R
        elif (90 < servo_position < 180) and US_dist <= 250:
            state = state_list[2]
            print("Current state is '{}".format(state))

    if state == 'Follow_Line':
        while True:
            ir_sens_readL = IR_sensorL.value()
            ir_sens_readC = IR_sensorC.value()
            ir_sens_readR = IR_sensorR.value()
            if ir_sens_readC == 1:
                move_forward()
                if ir_sens_readL == 1:


    #elif state == 'Follow_Wall':
        #print(state)    # Update to OLED

    if state == 'Follow_Line':

# Control signals
if dir_left == 'fwd':
    Motor_Left.set_forwards()
    else motor_left.set_backwards()
if dir_right == 'fwd':
    Motor_Right.set_forwards()
    else motor_right.set_backwards()

Motor_Left.duty(pwmL)
Motor_Right.duty(pwmR)

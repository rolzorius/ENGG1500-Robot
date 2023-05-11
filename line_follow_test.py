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


# ----------- ALL FUNCTIONS DEFINED HERE -------------------------------
# Sensors take initial values here
def sens_input():
    VCP.US_distF = ultrasonic_sensor.distance_mm()
    #VCP.rgb_prox = apds9960.prox.proximityLevel
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

# defines servo sweep function
def setServoAngle(angle):
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    range(0, 180, 15)
    pwm_servo.duty_u16(position)

def StraightLines():
    ir_sens_readC = IR_sensorC.value()
    ir_sens_readR = IR_sensorR.value()
    ir_sens_readL = IR_sensorL.value()
    move_forward()
    while ir_sens_readC == 1:
        move_forward()
        ir_sens_readC = IR_sensorC.value()
        ir_sens_readR = IR_sensorR.value()
        ir_sens_readL = IR_sensorL.value()
        # while ir_sens_readL == 0 and ir_sens_readC == 0 and ir_sens_readR == 0:
        #     Left_Motor.duty(35)
        #     Right_Motor.duty(35)

    while ir_sens_readR == 1 and ir_sens_readC == 0:
        ir_sens_readC = IR_sensorC.value()
        ir_sens_readR = IR_sensorR.value()
        Right_Motor.duty(50)
        Left_Motor.duty(0)
        # while ir_sens_readL == 0 and ir_sens_readC == 0 and ir_sens_readR == 0:
        #     Left_Motor.duty(35)
        #     Right_Motor.duty(35)

    while ir_sens_readL == 1 and ir_sens_readC == 0:
        ir_sens_readC = IR_sensorC.value()
        ir_sens_readR = IR_sensorR.value()
        ir_sens_readL = IR_sensorL.value()
        Left_Motor.duty(50)
        Right_Motor.duty(0)

    if ir_sens_readL == 0 and ir_sens_readC == 0 and ir_sens_readR == 0:
        Left_Motor.duty(40)
        Right_Motor.duty(40)

    return




# def line_follow(): # TODO this needs to be developed
#
#
#
#     # if VCP.ir_sens_readC == 1:
#     #     Left_Motor.set_forwards()
#     #     Right_Motor.set_forwards()
#     #     Left_Motor.duty(50)
#     #     Right_Motor.duty(50)
#
#     if VCP.ir_sens_readL == 1:
#         Left_Motor.set_forwards()
#         Right_Motor.set_forwards()
#         Left_Motor.duty(0)
#         Right_Motor.duty(50)
#
#     if VCP.ir_sens_readR == 1:
#         Left_Motor.set_forwards()
#         Right_Motor.set_forwards()
#         Left_Motor.duty(50)
#         Right_Motor.duty(0)
#
#     else:
#         Left_Motor.set_forwards()
#         Right_Motor.set_forwards()
#         Left_Motor.duty(50)
#         Right_Motor.duty(50)
#

# variables that need retained value in loops
dir_left = 0
dir_right = 0
servo_position = 0
goal_count = 0
direction = 0
action = 0
thru_count = 0


while True:
    ir_sens_readC = IR_sensorC.value()
    ir_sens_readR = IR_sensorR.value()
    ir_sens_readL = IR_sensorL.value()
    StraightLines()
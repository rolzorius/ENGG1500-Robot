# This file holds all pin values and constants used in calculations.
# It exists due to complexity of main.py and to aid in the troubleshooting of main.py.
# Will allow for easy manipulation of values throughout code instead of changing the same variable
# at multiple different points throughout the main.py file.
# -----------------------------------------------------------------------------------------------
from encoder import Encoder

#IR SENSOR PINS
IRCentrePin = 26        # central IR sensor Pin
IRRightPin = 27         # Right IR pin
IRLeftPin = 28          # Left IR pin


# ENCODER AND MOTOR PINS
encPinL = 18        # Left encoder pin
encPinR = 19        # Right encoder pin


# !! change these to appropriate names !!
LMP1 = 10             # Left motor pin
LMP2 = 11             # Left motor pin
LMP3 = 7             # Left motor pin

RMP1 = 9             # Right motor pin
RMP2 = 8             # Right motor pin
RMP3 = 6             # Right motor pin

# ULTRASONIC PINS
TRIGPin_1 = 3     # TRIG pin for US sensor 1
ECHOPin_1 = 2     # ECHO pin for US sensor 1

TRIGPin_2 = 5       # TRIG pin for US sensor 2
ECHOPin_2 = 4       # ECHO pin for US sensor 2

# SERVO PINS, PWM VALUE, ANGLE
servoPin = 15       # servo pin
servoPWM = 50       # servoPWM.freq

# OLED PINS
oledSDA = 12        # oled SDA pin
oledSCL = 13        # oled SCL pin

# RBG PINS
rgbSDA = 16
rgbSCL = 17


# MOTOR PWM CONSTANTS
RMpwm = 50      # R motor PWM at 50%
LMpwm = 50      # L motor PWM at 50%

Encoder_Left = encPinL
Encoder_Right = encPinR
enc = Encoder(Encoder_Left, Encoder_Right)
encCountL = enc.get_left()      # L encoder count
encCountR = enc.get_right()     # R encoder count
encAverage = (encCountL + encCountR)/2

# Motor Linearization calcs
basePWM = 35           # % of power to motor (PWM at 45%)
scalePWM = 3            # Scaling integer. Tested at 1, 2, 3, 4, 5 and 3 seems right.
LeftLinearPWM = basePWM + scalePWM * (encCountR - encCountL)      # L motor linearization
RightLinearPWM = basePWM + scalePWM * (encCountL - encCountR)     # R motor linearization

L_smladjPWM = 50        # PWM to motor for small adjustment function !!!!!! NEEDS TESTING AND OPTIMIZING
R_smladjPWM = 50        # PWM to motor for small adjustment function !!!!!! NEEDS TESTING AND OPTIMIZING

# Sensor variables
US_distL = 0
US_distF = 0
US_distR = 0

ir_sens_readL = 0
ir_sens_readC = 0
ir_sens_readR = 0

rgb_prox = 0            # This is reversed. So higher values indicate closer proximity
                        # If < 1 then it's not detecting anything i.e. no obstacle
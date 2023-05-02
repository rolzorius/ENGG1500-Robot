# This file holds all pin values and constants used in calculations.
# It exists due to complexity of main.py and to aid in the troubleshooting of main.py.
# Will allow for easy manipulation of values throughout code instead of changing the same variable
# at multiple different points throughout the main.py file.
# -----------------------------------------------------------------------------------------------
from encoder import Encoder


#IR SENSOR PINS
IRCentrePin = 26        # central IR sensor Pin
IRLeftPin = 27          # Left IR pin
IRRightPin = 29         # Right IR pin

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
TRIGPin = 3     # TRIG pin
ECHOPin = 2     # ECHO pin

# SERVO PINS, PWM VALUE, ANGLE
servoPin = 15       # servo pin
servoPWM = 50       # servoPWM.freq
angleConv = 8000 * (angle / 180) + 1000

# OLED PINS
oledSDA = 12        # oled SDA pin
oledSCL = 13        # oled SCL pin


# MOTOR PWM CONSTANTS
RMpwm = 50      # R motor PWM at 50%
LMpwm = 50      # L motor PWM at 50%


encCountL = enc.get_left()      # L encoder count
encCountR = enc.get_right()     # R encoder count
basePWM = 45            # % of power to motor (PWM at 45%)
scalePWM = 3            # Scaling integer. Tested at 1, 2, 3, 4, 5 and 3 seems right.

LeftLinearPWM = basePWM + scalePWM (encCountR - encCountL)      # L motor linearization
RightLinearPWM = basePWM + scalePWM (encCountL - encCountR)     # R motor linearization

L_smladjPWM = 50        # PWM to motor for small adjustment function !!!!!! NEEDS TESTING AND OPTIMIZING
R_smladjPWM = 50        # PWM to motor for small adjustment function !!!!!! NEEDS TESTING AND OPTIMIZING


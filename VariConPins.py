# This file holds all pin values and constants used in calculations.
# It exists due to complexity of motorcalc.py and to aid in the troubleshooting of motorcalc.py.
# Will allow for easy manipulation of values throughout code instead of changing the same variable
# at multiple different points throughout the motorcalc.py file.
# ----------------------------------------------------------------------------------------------
#IR SENSOR PINS
IRCentrePin = 26        # central IR sensor Pin
IRRightPin = 28         # Right IR pin
IRLeftPin = 27          # Left IR pin

# !! change these to appropriate names !!
LMP1 = 10             # Left motor pin
LMP2 = 11             # Left motor pin
LMP3 = 7             # Left motor pin

RMP1 = 9             # Right motor pin
RMP2 = 8             # Right motor pin
RMP3 = 6             # Right motor pin

# ULTRASONIC PINS
TRIGPin = 3     # TRIG pin for US sensor 1
ECHOPin = 2     # ECHO pin for US sensor 1

# SERVO PINS, PWM VALUE, ANGLE
servoPin = 15       # servo pin
servoPWM = 50       # servoPWM.freq

# OLED PINS
oledSDA = 12        # oled SDA pin
oledSCL = 13        # oled SCL pin

# RBG PINS
rgbSDA = 16
rgbSCL = 17

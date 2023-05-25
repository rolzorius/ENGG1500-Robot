from time import sleep
from machine import I2C, Pin, PWM
from motor import Motor
import VariConPins as VCP
from ultrasonic import sonic
from ssd1306 import SSD1306_I2C

# Sensor variables
US_distL = 0
US_distF = 0
US_distR = 0

ir_sens_readL = 0
ir_sens_readC = 0
ir_sens_readR = 0

# Initialise the motors and encoders
Left_Motor = Motor("left", VCP.LMP1, VCP.LMP2, VCP.LMP3)
Right_Motor = Motor("right", VCP.RMP1, VCP.RMP2, VCP.RMP3)

# define ultrasonic 1
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

# Define OLED object
i2c = I2C(0, sda=Pin(VCP.oledSDA), scl=Pin(VCP.oledSCL))
oled = SSD1306_I2C(128, 64, i2c)


# ----------- ALL FUNCTIONS DEFINED HERE -------------------------------
# Sensors take initial values here
def setServoAngle(angle):
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    range(0, 180, 15)
    pwm_servo.duty_u16(position)


def US_Update():
    US_distF = ultrasonic_sensor.distance_mm()
    US_distL = ultrasonic_sensor.distance_mm()
    US_distR = ultrasonic_sensor.distance_mm()
    # US_distFR = ultrasonic_sensor.distance_mm()
    # US_distFL = ultrasonic_sensor.distance_mm()
    for pos in range(0, 180, 15):
        setServoAngle(pos)  # Set servo to desired angle
        if pos == 15:
            sleep(0.2)
            pwm_servo.duty_u16(0)
            sleep(0.2)
            US_distR = ultrasonic_sensor.distance_mm()
        elif pos == 90:
            sleep(0.2)
            pwm_servo.duty_u16(0)
            sleep(0.2)
            US_distF = ultrasonic_sensor.distance_mm()
        elif pos == 165:
            sleep(0.2)
            pwm_servo.duty_u16(0)
            sleep(0.2)
            US_distL = ultrasonic_sensor.distance_mm()
    setServoAngle(90)
    sleep(1)
    return US_distF, US_distL, US_distR

def IR_Update():
    ir_sens_readC = IR_sensorC.value()
    ir_sens_readL = IR_sensorL.value()
    ir_sens_readR = IR_sensorR.value()

    return ir_sens_readC, ir_sens_readL, ir_sens_readR
# # Defines OLED update
def oled_update():
    oled.init_display()
    oled.text("State = {}".format(state), 0, 0)
    oled.text("L Dist = {}".format(US_distL), 0, 14)
    oled.text("F Dist = {}".format(US_distF), 0, 28)
    oled.text("R Dist = {}".format(US_distR), 0, 42)
    oled.show()

# !!! This section defines all movement functions
def move_backward():
    Left_Motor.set_backwards()
    Right_Motor.set_backwards()
    Right_Motor.duty(51)
    Left_Motor.duty(55)
    sleep(0.2)
    Right_Motor.duty(0)
    Left_Motor.duty(0)

def full_stop():
    Left_Motor.duty(0)
    Right_Motor.duty(0)

def wall_follow():
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()
    Left_Motor.duty(0)
    Right_Motor.duty(0)
    US_distF, US_distL, US_distR = US_Update()
    oled_update()
    if US_distL >= 75 and US_distR >= 75:
        crawl_S()
    elif US_distR < 75:
        Right_Motor.duty(50)
        Left_Motor.duty(0)
        sleep(0.2)
        Right_Motor.duty(0)
    elif US_distL < 75:
        Left_Motor.duty(50)
        Right_Motor.duty(0)
        sleep(0.2)
        Left_Motor.duty(0)

    return

def crawl_S():
    US_distF = ultrasonic_sensor.distance_mm()
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()
    Right_Motor.duty(51)
    Left_Motor.duty(0)
    sleep(0.2)
    Right_Motor.duty(0)
    Left_Motor.duty(51)
    sleep(0.2)
    Right_Motor.duty(0)
    Left_Motor.duty(0)

def crawl_L():
    Right_Motor.set_forwards()
    Right_Motor.duty(50)
    sleep(0.07)
    Right_Motor.duty(0)
    sleep(0.07)

def crawl_R():
    Left_Motor.set_forwards()
    Left_Motor.duty(0)
    sleep(0.07)
    Left_Motor.duty(50)
    sleep(0.07)


def line_follow():
    US_distF = ultrasonic_sensor.distance_mm()
    ir_sens_readC = IR_sensorC.value()
    ir_sens_readL = IR_sensorL.value()
    ir_sens_readR = IR_sensorR.value()

    if ir_sens_readC == 1 and ir_sens_readL == 0 and ir_sens_readR == 0:
        Right_Motor.set_forwards()
        Left_Motor.set_forwards()
        Right_Motor.duty(50)
        Left_Motor.duty(50)
        sleep(0.1)
        Right_Motor.duty(0)
        Left_Motor.duty(0)
        sleep(0.1)

    elif ir_sens_readL == 1 and ir_sens_readC == 0:
        Right_Motor.set_forwards()
        Left_Motor.set_forwards()
        Right_Motor.duty(50)
        Left_Motor.duty(0)
        sleep(0.1)
        Right_Motor.duty(0)
        Left_Motor.duty(0)
        sleep(0.1)
        # Right_Motor.duty(50)
        # Left_Motor.duty(0)
        # sleep(0.05)

    elif ir_sens_readR == 1 and ir_sens_readC == 0:
        Right_Motor.set_forwards()
        Left_Motor.set_forwards()
        Right_Motor.duty(0)
        Left_Motor.duty(50)
        sleep(0.1)
        Right_Motor.duty(0)
        Left_Motor.duty(0)
        sleep(0.1)
        # Right_Motor.duty(0)
        # Left_Motor.duty(50)
        # sleep(0.05)

    elif ir_sens_readC == 1 and ir_sens_readL == 1 and ir_sens_readR == 1:
        Right_Motor.set_forwards()
        Right_Motor.duty(50)
        sleep(0.12)
        Right_Motor.duty(0)
        sleep(0.12)
    elif ir_sens_readC and ir_sens_readR and ir_sens_readL == 0:
        US_distF = ultrasonic_sensor.distance_mm()
        crawl_R()
    elif ir_sens_readC and ir_sens_readL and ir_sens_readR == 0:
        US_distF = ultrasonic_sensor.distance_mm()
        crawl_L()

    if 150 <= US_distF <= 400:
        US_distF = ultrasonic_sensor.distance_mm()
        crawl_S()
        if 10 < US_distF < 100:
            full_stop()
    return


# # ----------------------------- state machine code starts here ----------------------------- # #

# Declare states
state_list = {0: 'Start', 1: 'Driving', 2: 'Wall', 3: 'Line'}

# Default state when init is 'Start'
state = state_list[0]
oled_update()

while True:  # # ----- States and transition conditions ----- # #
    if state == 'Start':
        ir_sens_readC, ir_sens_readL, ir_sens_readR = IR_Update()
        full_stop()
        sleep(1)  # 2 seconds of nothing then
        state = state_list[1]  # changes state from Stopped to Driving

        oled_update()

    elif state == 'Driving':  # state 1
        oled_update()

        ir_sens_readC = IR_sensorC.value()
        ir_sens_readL = IR_sensorL.value()
        ir_sens_readR = IR_sensorR.value()

        US_distF, US_distL, US_distR = US_Update()

        if 0 < US_distR <= 300 and 0 < US_distL <= 300:
            while 0 < US_distF < 300:
                US_distF, US_distL, US_distR = US_Update()
                if US_distR > US_distL:
                    Left_Motor.set_backwards()
                    Left_Motor.duty(75)
                    sleep(0.1)
                    Left_Motor.duty(0)
                    sleep(0.5)
                    Right_Motor.set_backwards()
                    Right_Motor.duty(75)
                    sleep(0.1)
                    Right_Motor.duty(0)
                    sleep(0.5)
                if US_distL > US_distR:
                    Right_Motor.set_backwards()
                    Right_Motor.duty(75)
                    sleep(0.1)
                    Right_Motor.duty(0)
                    sleep(0.5)
                    Left_Motor.set_backwards()
                    Left_Motor.duty(75)
                    sleep(0.1)
                    Left_Motor.duty(0)
                    sleep(0.5)

                if 200 < US_distF < 300:
                    Left_Motor.set_forwards()
                    Right_Motor.set_backwards()
                    Left_Motor.duty(50)
                    Right_Motor.duty(50)
                    sleep(0.8)
                    Right_Motor.duty(0)
                    Left_Motor.duty(0)

            if US_distR > US_distL:
                Left_Motor.set_forwards()
                Left_Motor.duty(75)
                sleep(0.1)
                Left_Motor.duty(0)
                sleep(0.5)
                Right_Motor.set_forwards()
                Right_Motor.duty(75)
                sleep(0.1)
                Right_Motor.duty(0)
                sleep(0.5)
            if US_distL > US_distR:
                Right_Motor.set_forwards()
                Right_Motor.duty(75)
                sleep(0.1)
                Right_Motor.duty(0)
                sleep(0.5)
                Left_Motor.set_forwards()
                Left_Motor.duty(75)
                sleep(0.1)
                Left_Motor.duty(0)
                sleep(0.5)

        if ir_sens_readL or ir_sens_readC or ir_sens_readR:
            state = state_list[3]  # SWITCH TO LINE_FOLLOW
            oled_update()

        else:
            crawl_S()

    elif state == 'Wall':  # state 2
        ir_sens_readC, ir_sens_readL, ir_sens_readR = IR_Update()
        oled_update()
        wall_follow()
        if ir_sens_readL or ir_sens_readC or ir_sens_readR:
            state = state_list[3]  # SWITCH TO LINE_FOLLOW
            oled_update()
        elif 50 < US_distF <= 150:
            full_stop()

    elif state == 'Line':   # state 3
        ir_sens_readC, ir_sens_readL, ir_sens_readR = IR_Update()
        US_distF = ultrasonic_sensor.distance_mm()

        Left_Motor.set_forwards()
        Right_Motor.set_forwards()

        line_follow()

        if ir_sens_readC == 0 and ir_sens_readL == 0 and ir_sens_readR == 0:
            Left_Motor.duty(0)
            Right_Motor.duty(0)
            US_distF, US_distL, US_distR = US_Update()
            if 0 < US_distR <= 300 and 0 < US_distL <= 300:
                state = state_list[2]
                oled_update()
            # elif US_distL < 200:
            #     Left_Motor.duty(50)
            #     sleep(0.5)
            #     Left_Motor.duty(0)
            # elif US_distR < 200:
            #     Right_Motor.duty(50)
            #     sleep(0.5)
            #     Right_Motor.duty(0)
            else:
                move_backward()



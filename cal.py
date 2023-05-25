from motor import Motor
from encoder import Encoder
from machine import Pin, PWM, ADC
from ultrasonic import sonic
from time import sleep
from machine import Pin, I2C
from ssd1306 import SSD1306_I2C


i2c = I2C(0, sda=Pin(12), scl=Pin(13))
oled = SSD1306_I2C(128, 64, i2c)

# Ultrasonic
TRIG = 3
ECHO = 2
ultrasonic_sensor = sonic(TRIG, ECHO)
dist_L = 0
dist_R = 0
dist_F = 0
print("Initialised")

# Servo
pwm_servo = PWM(Pin(15))
pwm_servo.freq(50)
# Roundabout Counter
Rcount = 0
# Motors and encoders
Left_Motor = Motor("left", 10, 11, 7)  # where x = Pins tba
Right_Motor = Motor("right", 9, 8, 6)  # where x = Pins tba
Encoder_Left = 19
Encoder_Right = 18
enc = Encoder(Encoder_Left, Encoder_Right)
encCountL = 0
encCountR = 0
encAverage = 0
pwmL = 0
pwmR = 0

# IR Sensors
IR_sensorC = Pin(26, Pin.IN)
IR_sensorR = Pin(27, Pin.IN)
IR_sensorL = Pin(28, Pin.IN)
ir_sens_readC = 0
ir_sens_readL = 0
ir_sens_readR = 0


def move_forward():
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()
    Right_Motor.duty(48)
    Left_Motor.duty(0)
    sleep(0.2)
    Right_Motor.duty(0)
    Left_Motor.duty(51)
    sleep(0.2)


def move_forward2():
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()
    Left_Motor.duty(60)
    Right_Motor.duty(60)
    # balanced PWM output using encoder
    encCountL = enc.get_left()
    encCountR = enc.get_right()
    pwmL = + 3 * (encCountR - encCountL)
    pwmR = 45 + 3 * (encCountL - encCountR)
    Left_Motor.duty(pwmL)
    Right_Motor.duty(pwmR)


def move_backward():  # Used any time the bot MOVES BACKWARD ONLY
    Left_Motor.set_backwards()
    Right_Motor.set_backwards()
    encCountL = enc.get_left()
    encCountR = enc.get_right()
    pwmL = 45 + 3 * (encCountR - encCountL)
    pwmR = 45 + 3 * (encCountL - encCountR)
    Left_Motor.duty(pwmL)


def setServoAngle(angle):
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    range(0, 180, 30)
    pwm_servo.duty_u16(position)


def SlowServoSweep():
    Left_Motor.duty(0)  # |
    Right_Motor.duty(0)  # |
    pwm_servo.duty_u16(0)  # |
    sleep(1)  # |
    setServoAngle(90)  # |
    dist_F = ultrasonic_sensor.distance_mm()  # |
    sleep(0.5)  # |
    pwm_servo.duty_u16(0)  # |
    sleep(1)  # |
    setServoAngle(15)  # |
    sleep(0.5)  # |
    dist_R = ultrasonic_sensor.distance_mm()  # |
    pwm_servo.duty_u16(0)  # |
    sleep(2)  # |
    setServoAngle(170)  # |
    sleep(0.5)  # |
    dist_L = ultrasonic_sensor.distance_mm()
    return dist_F, dist_L, dist_R


def Welcome_Home():  # Runs after it has Returned to where it Believes the Garage is. Should play after line eneds
    while True:
        setServoAngle(90)
        dist_F = ultrasonic_sensor.distance_mm()
        while dist_F < 350:  # Checks there is a wall in front of the goggles
            SlowServoSweep()
            dist_F, dist_L, dist_R = SlowServoSweep()
            while dist_R and dist_L < 350:  # Checks there is a wall on either side of the bot, If true Garage Found
                Left_Motor.set_forwards()
                Right_Motor.set_forwards()
                Left_Motor.duty(50)
                Right_Motor.duty(50)
                setServoAngle(90)
                dist_F = ultrasonic_sensor.distance_mm()
                if dist_F < 150:
                    Left_Motor.duty(0)
                    Right_Motor.duty(0)
                    return

            Left_Motor.set_forwards()
            Right_Motor.set_forwards()
            Left_Motor.duty(50)
            Right_Motor.duty(50)


def ComingHome():
    SlowServoSweep()
    dist_F, dist_L, dist_R = SlowServoSweep()
    ir_sens_readC = IR_sensorC.value()
    ir_sens_readR = IR_sensorR.value()
    ir_sens_readL = IR_sensorL.value()
    while ir_sens_readC == 1 and ir_sens_readL == 0 and ir_sens_readR == 0:
        Left_Motor.set_forwards()
        Right_Motor.set_forwards()
        Left_Motor.duty(50)
        Right_Motor.duty(50)
        ir_sens_readC = IR_sensorC.value()
        ir_sens_readR = IR_sensorR.value()
        ir_sens_readL = IR_sensorL.value()
        SlowServoSweep()
        dist_F, dist_L, dist_R = SlowServoSweep()
    while ir_sens_readR == 1 and ir_sens_readC == 0:
        ir_sens_readC = IR_sensorC.value()
        ir_sens_readR = IR_sensorR.value()
        SlowServoSweep()
        dist_F, dist_L, dist_R = SlowServoSweep()
        Right_Motor.set_forwards()
        Left_Motor.set_backwards()
        Right_Motor.duty(50)
        Left_Motor.duty(50)
    while ir_sens_readL == 1 and ir_sens_readC == 0:
        ir_sens_readC = IR_sensorC.value()
        ir_sens_readL = IR_sensorL.value()
        dist_F, dist_L, dist_R = SlowServoSweep()
        Right_Motor.set_backwards()
        Left_Motor.set_forwards()
        Right_Motor.duty(50)
        Left_Motor.duty(50)
    while 0 < dist_F <= 300:
        Welcome_Home()
    return


def Spin360():  # Turns around
    Left_Motor.set_forwards()
    Right_Motor.set_backwards()
    Left_Motor.duty(50)
    Right_Motor.duty(50)
    sleep(0.75)
    Right_Motor.duty(0)
    Left_Motor.duty(0)

    return


def StraightLines():
    ir_sens_readC = IR_sensorC.value()
    ir_sens_readR = IR_sensorR.value()
    ir_sens_readL = IR_sensorL.value()
    while ir_sens_readC == 1 and ir_sens_readL == 0 and ir_sens_readR == 0:
        Right_Motor.set_forwards()
        Left_Motor.set_forwards()
        Right_Motor.duty(50)
        Left_Motor.duty(50)

        ir_sens_readC = IR_sensorC.value()
        ir_sens_readR = IR_sensorR.value()
        ir_sens_readL = IR_sensorL.value()
    while ir_sens_readR == 1 and ir_sens_readC == 0:
        ir_sens_readC = IR_sensorC.value()
        ir_sens_readR = IR_sensorR.value()
        Right_Motor.set_forwards()
        Left_Motor.set_backwards()
        Right_Motor.duty(50)
        Left_Motor.duty(50)
    while ir_sens_readL == 1 and ir_sens_readC == 0:
        ir_sens_readC = IR_sensorC.value()
        ir_sens_readL = IR_sensorL.value()
        Right_Motor.set_backwards()
        Left_Motor.set_forwards()
        Right_Motor.duty(50)
        Left_Motor.duty(50)
        # while ir_sens_readL == 1 and ir_sens_readR == 1:
        #     Right_Motor.set_forwards()
        #     Left_Motor.set_forwards()
        #     Left_Motor.duty(0)
        #     Right_Motor.duty(0)
        return


def Roundabout(
        Rcount):  # count the amount of roundabouts that it has seen, and respond accordingly    # will get us to the first roundabout and back
    if Rcount == 1:
        Right_Motor.set_forwards()
        Left_Motor.set_forwards()
        Right_Motor.duty(50)
        Left_Motor.duty(50)
        sleep(0.1)

    elif Rcount == 2:
        Right_Motor.set_forwards()
        Left_Motor.set_forwards()
        Right_Motor.duty(50)
        Left_Motor.duty(50)
        sleep(0.1)

    elif Rcount == 3:
        Spin360()
        ir_sens_readC = IR_sensorC.value()
        ir_sens_readR = IR_sensorR.value()
        ir_sens_readL = IR_sensorL.value()
        while Rcount == 3:
            while ir_sens_readL == 0 or ir_sens_readR == 0 or ir_sens_readC == 0:
                Right_Motor.set_forwards()
                Left_Motor.set_forwards()
                Right_Motor.duty(50)
                Left_Motor.duty(50)

            ComingHome()

    return


# ____________________________________----------------------------------STATE____--------------------------------------


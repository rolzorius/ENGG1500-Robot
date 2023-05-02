from time import sleep
from motor import Motor
from encoder import Encoder
from machine import Pin, PWM, ADC
from ultrasonic import sonic
from machine import Pin, I2
from ssd1306 import SSD1306_I2C


sleep(3)  # sleep for 3 seconds BEFORE doing anything

# Create Motor and Encoder Objects
Left_Motor = Motor("left", 10, 11, 7)  # where x = Pins tba
Right_Motor = Motor("right", 9, 8, 6)  # where x = Pins tba
Encoder_Left = 18
Encoder_Right = 19
enc = Encoder(Encoder_Left, Encoder_Right)

# define IR sensors and LED objects
IR_sensorC = ADC(Pin(26))
IR_sensorR = ADC(Pin(29))
IR_sensorL = ADC(Pin(27))
LED_green = Pin("LED", Pin.OUT)
ir_sens_readC = 0
ir_sens_readL = 0
ir_sens_readR = 0
print(ir_sens_readR, ir_sens_readL, ir_sens_readC)

# define ultrasonic sensor object
TRIG = 3
ECHO = 2
ultrasonic_sensor = sonic(TRIG, ECHO)

dist_L = 0
dist_R = 0
dist_F = 0

# define servo object
pwm_servo = PWM(Pin(15))
pwm_servo.freq(50)

# define OLED
i2c = I2C(0, sda=Pin(12), scl=Pin(13))
oled = SSD1306_I2C(128, 64, i2c)

def move_forward():
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()

    # balanced PWM output using encoder
    encCountL = enc.get_left()
    encCountR = enc.get_right()
    pwmL = 45 + 3 * (encCountR - encCountL)
    pwmR = 45 + 3 * (encCountL - encCountR)
    Left_Motor.duty(pwmL)
    Right_Motor.duty(pwmR)


def line_follow():
    ir_sens_read_C = IR_sensorC.read_u16() / 65535
    ir_sens_read_L = IR_sensorL.read_u16() / 65535
    ir_sens_read_R = IR_sensorR.read_u16() / 65535
    OLED_init()
    if ir_sens_read_C >= 0.8:
        move_forward()
        if ir_sens_read_L > ir_sens_read_R < 0.8:
            while True:
                Right_Motor.duty(0)
                Left_Motor.duty(50)
                if ir_sens_read_C < 0.2:
                    break
        elif ir_sens_read_R > ir_sens_read_L < 0.8:
            while True:
                Right_Motor.duty(50)
                Left_Motor.duty(0)
                if ir_sens_read_C < 0.2:
                    break
        # elif ir_sens_readR > 0.8: Turning bot onto right hand branch

        # elif ir_sens_readL > 0.8: Turning bot onto Left hand branch


def setServoAngle(angle):
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    range(0, 180, 30)
    pwm_servo.duty_u16(position)


def wall_follow():
    Left_Motor.duty(0)
    Right_Motor.duty(0)
    dist_F = ultrasonic_sensor.distance_mm()
    OLED_init()
    sleep(2)
    if dist_F < 100:
        setServoAngle(15)
        sleep(0.5)
        dist_R = ultrasonic_sensor.distance_mm()
        OLED_init()
        pwm_servo.duty_u16(0)
        sleep(2)
        setServoAngle(170)
        sleep(0.5)
        dist_L = ultrasonic_sensor.distance_mm()
        OLED_init()
        pwm_servo.duty_u16(0)
        sleep(2)
        setServoAngle(90)
        sleep(0.5)
        pwm_servo.duty_u16(0)
        sleep(2)
        if dist_R > dist_L:
            enc.clear_count()
            while enc.get_left() <= 10:
                print(enc.get_left())
                Left_Motor.set_forwards()
                Left_Motor.duty(50)
            while enc.get_right() <= 10:
                print(enc.get_right)
                Right_Motor.set_forwards()
                Right_Motor.duty(50)
        elif dist_L > dist_R:
            enc.clear_count()
            while enc.get_right() <= 10:
                Right_Motor.duty(50)
                print(enc.get_right())
            while enc.get_left() <= 10:
                Left_Motor.duty(50)
                print(enc.get_left())


def OLED_init():
    oled.text("IRL={}".format(ir_sens_readL), 0, 0)
    oled.text("IRC={}".format(ir_sens_readC), 0, 20)
    oled.text("IRR={}".format(ir_sens_readR), 0, 40)
    oled.text("USL={}".format(dist_L), 56, 0)
    oled.text("USC={}".format(dist_F), 56, 20)
    oled.text("USR={}".format(dist_R), 56, 40)
    oled.show()  # Note: 0,0 top left, 128, 64 bottom right. 128 X 64 CORDs


while True:
    ir_sens_read_C = IR_sensorC.read_u16() / 65535 # Remove the magic numbers
    ir_sens_read_L = IR_sensorL.read_u16() / 65535
    ir_sens_read_R = IR_sensorR.read_u16() / 65535
    OLED_init()
    if ir_sens_readC and ir_sens_readR and ir_sens_readL < 0.9: #what are we trying to do here?
        #What does this do?
        #Who wrote it?
        #When?
        #What does each variable do?
        setServoAngle(0)
        dist_L = ultrasonic_sensor.distance_mm()
        sleep(0.2)
        setServoAngle(180)
        dist_R = ultrasonic_sensor.distance_mm()
        sleep(0.2)
        OLED_init()
        if dist_R and dist_L < 175:
            while ir_sens_readC and ir_sens_readR and ir_sens_readL < 0.9:
                wall_follow()
        else:
            enc.clear_count()
            while enc.get_left() and enc.get_right() < 30:
                move_forward()
    elif ir_sens_readC > 0.8:
        print('Yes')
        line_follow()

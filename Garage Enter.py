from motor import Motor
from encoder import Encoder
from machine import Pin, PWM, I2C, ADC
from ultrasonic import sonic
from time import sleep
from APDS9960LITE import APDS9960LITE
# Motors
Left_Motor = Motor("left", 10, 11, 7)  # where x = Pins tba
Right_Motor = Motor("right", 9, 8, 6)  # where x = Pins tba
Encoder_Left = 19
Encoder_Right = 18
enc = Encoder(Encoder_Left, Encoder_Right)
encCountL = 0
encCountR = 0
pwmL = 0
pwmR = 0

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

# RGB Sensor
i2c = I2C(0, scl=Pin(17), sda=Pin(16))
apds9960 = APDS9960LITE(I2C)
apds9960.prox.enableSensor()
sleep(0.1)
RGB_Measure = 0

# IR Sensors
IR_sensorC = ADC(Pin(26))
IR_sensorR = ADC(Pin(28))
IR_sensorL = ADC(Pin(27))
ir_sens_readC = 0
ir_sens_readL = 0
ir_sens_readR = 0


def setServoAngle(angle):
    position = int(8000 * (angle / 180) + 1000)  # Convert angle into [1000, 9000]
    range(0, 180, 30)
    pwm_servo.duty_u16(position)

def Welcome_Home():     # Runs after it has Returned to where it Believes the Garage is. Should play after line eneds
    dist_F = ultrasonic_sensor.distance_mm()
    setServoAngle(90)
    encLeft = 0
    encRight = 0
    while dist_F < 200:   # Checks there is a wall in front of the goggles
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
        if dist_R and dist_L < 200:     # Checks there is a wall on either side of the bot, If true Garage Found
            RGB_Measure = proximity_measurement = apds9960.prox.proximityLevel

            if RGB_Measure > 10:     # Value must be finalised, Checking to see if it will hit the wall.
                Left_Motor.set_forwards()
                Right_Motor.set_forwards()

                # driving forwards into the garage
                encCountL = enc.get_left()
                encCountR = enc.get_right()
                pwmL = 45 + 3 * (encCountR - encCountL)
                pwmR = 45 + 3 * (encCountL - encCountR)
                Left_Motor.duty(pwmL)
                Right_Motor.duty(pwmR)
                RGB_Measure = proximity_measurement = apds9960.prox.proximityLevel


    # Drives Forwards while it cant see a line or a wall
    Left_Motor.set_forwards()
    Right_Motor.set_forwards()

    # balanced PWM output using encoder
    encCountL = enc.get_left()
    encCountR = enc.get_right()
    pwmL = 45 + 3 * (encCountR - encCountL)
    pwmR = 45 + 3 * (encCountL - encCountR)
    Left_Motor.duty(pwmL)
    Right_Motor.duty(pwmR)
    RGB_Measure = proximity_measurement = apds9960.prox.proximityLevel



def Leaving_Home():     # first thing that Runs when the Bot starts
    dist_F = ultrasonic_sensor.distance_mm()    # Find Front Wall
    setServoAngle(90)
    encLeft = 0
    encRight = 0

    setServoAngle(15)   # Find Right Wall
    sleep(0.5)
    dist_R = ultrasonic_sensor.distance_mm()

    pwm_servo.duty_u16(0)   #Find left Wall
    sleep(2)
    setServoAngle(170)
    sleep(0.5)
    dist_L = ultrasonic_sensor.distance_mm()

    pwm_servo.duty_u16(0)   # go back to facing front
    sleep(2)
    setServoAngle(90)
    sleep(0.5)
    pwm_servo.duty_u16(0)

    ir_sens_readC = IR_sensorC.value()
    if dist_F and dist_L and dist_R < 250:  # Checks there is a wall in front of the goggles, If true, facing backwards
        setServoAngle(90)
        dist_F = ultrasonic_sensor.distance_mm()
        setServoAngle(15)  # Find Right Wall
        sleep(0.5)
        dist_R = ultrasonic_sensor.distance_mm()

        pwm_servo.duty_u16(0)  # Find left Wall
        sleep(2)
        setServoAngle(170)
        sleep(0.5)
        dist_L = ultrasonic_sensor.distance_mm()

        pwm_servo.duty_u16(0)  # go back to facing front
        sleep(2)
        setServoAngle(90)
        sleep(0.5)
        pwm_servo.duty_u16(0)
        if dist_F and dist_L and dist_R != 0:   # While it reads a value for the Garage
            Left_Motor.set_backwards()      # Books it backwards until it is out of the garage
            Right_Motor.set_backwards()

            # balanced PWM output using encoder
            encCountL = enc.get_left()
            encCountR = enc.get_right()
            pwmL = 45 + 3 * (encCountR - encCountL)
            pwmR = 45 + 3 * (encCountL - encCountR)
            Left_Motor.duty(pwmL)
            Right_Motor.duty(pwmR)
            dist_F = ultrasonic_sensor.distance_mm()    # Find Front wall

            setServoAngle(15)  # Find Right Wall
            sleep(0.5)
            dist_R = ultrasonic_sensor.distance_mm()

            pwm_servo.duty_u16(0)  # Find left Wall
            sleep(2)
            setServoAngle(170)
            sleep(0.5)
            dist_L = ultrasonic_sensor.distance_mm()

            pwm_servo.duty_u16(0)  # go back to facing front
            sleep(2)
            setServoAngle(90)
            sleep(0.5)
            pwm_servo.duty_u16(0)

        else:
            Left_Motor.set_forwards()
            Right_Motor. set_forwards()
            #SPIN BOT!!!!!!!!!!!!!!!!!!!!
    else:       # Facing forwards
        ir_sens_readC = IR_sensorC.value()
        if ir_sens_readC == 1:
            encCountL = enc.get_left()
            encCountR = enc.get_right()
            pwmL = 45 + 3 * (encCountR - encCountL)
            pwmR = 45 + 3 * (encCountL - encCountR)
            Left_Motor.duty(pwmL)
            Right_Motor.duty(pwmR)

        else:
            print('Line Follow')

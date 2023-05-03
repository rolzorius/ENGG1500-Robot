from machine import Pin
from time import sleep
import VariConPins as VCP

# define IR sensors and LED objects
IR_sensorL = Pin(VCP.IRLeftPin, Pin.IN)
IR_sensorC = Pin(VCP.IRCentrePin, Pin.IN)
IR_sensorR = Pin(VCP.IRRightPin, Pin.IN)

LED_green = Pin("LED", Pin.OUT)
ir_sens_readC = IR_sensorC.value()
ir_sens_readL = IR_sensorL.value()
ir_sens_readR = IR_sensorR.value()


# Define OLED object and function
i2c = I2C(0, sda=Pin(VCP.oledSDA), scl=Pin(VCP.oledSCL))
oled = SSD1306_I2C(128, 64, i2c)

def OLED_init():
    oled.text("IRL={}".format(ir_sens_readL), 0, 0)
    oled.text("IRC={}".format(ir_sens_readC), 0, 20)
    oled.text("IRR={}".format(ir_sens_readR), 0, 40)
    oled.text("USL={}".format(dist_L), 56, 0)
    oled.text("USC={}".format(dist_F), 56, 20)
    oled.text("USR={}".format(dist_R), 56, 40)
    oled.show()  # Note: 0,0 top left, 128, 64 bottom right. 128 X 64 CORDs

OLED_init()
print(ir_sens_readL, ir_sens_readC, ir_sens_readR)

# Control loop

while True:
    ir_sens_readC = IR_sensorC.value()
    ir_sens_readL = IR_sensorL.value()
    ir_sens_readR = IR_sensorR.value()
    OLED_init()
    print("Left IR sensor reads '{}', Centre IR sensor reads '{}', Right IR sensor reads '{}.".format
          (ir_sens_readL, ir_sens_readC, ir_sens_readR))
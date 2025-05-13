from pymycobot.mycobot import MyCobot
import time

mc = MyCobot("COM3", 115200)
#time.sleep(1)
mc.set_digital_output(2,0)
#mc.set_basic_output(2,0)
#time.sleep(1)
mc.set_pin_mode(2,0)

i = 0
while mc.get_digital_input(2) < 1:
    print(mc.get_digital_input(2))
    print(mc.get_basic_input(2))
    i = i+1
    if i > 15:
        break

time.sleep(1)

mc.set_digital_output(2,1)


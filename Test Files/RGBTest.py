from pymycobot.build.lib.pymycobot.utils import get_port_list
from pymycobot.mypalletizer import MyPalletizer
from pymycobot import MyCobot, utils, Angle, Coord
#from controller import Robot

#from pymycobot import PI_PORT, PI_BAUD      # When using the Raspberry Pi version of mycobot, you can refer to these two variables to initialize MyCobot, if not, you can omit this line of code
import time
#The above codes are required to be written, which means importing the project package

# MyCobot class initialization requires two parameters:
#   The first is the serial port string, such as:
#       linux:  "/dev/ttyUSB0"
#          or "/dev/ttyAMA0"
#       windows: "COM3"
#   The second is the baud rate::
#       M5 version is:  115200
#
#    Example:
#       mycobot-M5:
#           linux:
#              mc = MyCobot("/dev/ttyUSB0", 115200)
#          or mc = MyCobot("/dev/ttyAMA0", 115200)
#           windows:
#              mc = MyCobot("COM3", 115200)
#       mycobot-raspi:
#           mc = MyCobot(PI_PORT, PI_BAUD)
#
# Initiate MyCobot
# Create object code here for Windows version

mc = MyCobot("COM3", 115200)

i = 7
#loop 7 times
while i > 0:
    mc.set_color(0,0,255) #blue light on
    time.sleep(1)    #wait for 2 seconds
    mc.set_color(255,0,0) #red light on
    time.sleep(1)    #wait for 2 seconds
    mc.set_color(0,255,0) #green light on
    time.sleep(1)    #wait for 2 seconds
    print(i)
    i = i - 1

"""
  # initiate MyPalletizer
    mc = MyPalletizer("COM3", 115200)

    #     MyPalletizer initiation requires two parameters:
    #   The first is the serial port string, such as:
    #       linux:  "/dev/ttyUSB0"
    #          or "/dev/ttyAMA0"
    #       windows: "COM3"
    #   The second is the baud rate::
    #       M5 version is:  115200
    #
    #    Example:
    #       MyPalletizer-M5:
    #           linux:
    #              mc = MyPalletizer("/dev/ttyUSB0", 115200)
    #          or mc = MyPalletizer("/dev/ttyAMA0", 115200)
    #           windows:
    #              mc = MyPalletizer("COM3", 115200)

    i = 7
    # loops 7 times
    while i > 0:
        mc.set_color(0, 0, 255)  # blue light on
        time.sleep(2)  # wait for 2 seconds
        mc.set_color(255, 0, 0)  # red light on
        time.sleep(2)  # wait for 2 seconds
        mc.set_color(0, 255, 0)  # green light on
        time.sleep(2)  # wait for 2 seconds
        i -= 1
"""

exit(0)
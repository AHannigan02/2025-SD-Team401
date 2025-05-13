"""my controller description."""

from controller import *

robot = Robot()

timestep = int(robot.getBasicTimeStep())

led = robot.getDevice('ledName')
distanceSensor = robot.getDevice('distanceSensorName')
distanceSensor.enable(timestep)

while robot.step(timestep) != -1:
  # Read the sensors, like:
  val = distanceSensor.getValue()

  # Process sensor data here

  # Enter here functions to send actuator commands, like:
  led.set(1)

# Enter here exit cleanup code
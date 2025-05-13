from controller import Robot

TIME_STEP = 32

robot = Robot()

motor12 = robot.getMotor("joint2_to_joint1")
sensor12 = robot.getPositionSensor("joint2_to_joint1_sensor")

motor6o6 = robot.getMotor("joint6output_to_joint6")
sensor6o6 = robot.getPositionSensor("joint6output_to_joint6_sensor")

motor12.setPosition(float('inf'))

motor6o6.setPosition(float('inf'))

motor12.setVelocity(0.0)

motor6o6.setVelocity(0.0)

sensor12.enable(TIME_STEP)

sensor6o6.enable(TIME_STEP)

"""
left_sensor = robot.getDevice("left_sensor")
right_sensor = robot.getDevice("right_sensor")
left_sensor.enable(TIME_STEP)
right_sensor.enable(TIME_STEP)

left_motor = robot.getDevice("left_motor")
right_motor = robot.getDevice("right_motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
"""
while robot.step(TIME_STEP) != -1:

    # read sensors
    pos12 = sensor12.getValue()
    pos6o6 = sensor6o6.getValue()

    # compute behavior (user functions)
    #left = compute_left_speed(left_dist, right_dist)
    #right = compute_right_speed(left_dist, right_dist)

    # actuate wheel motors
    motor12.setVelocity(10)
    motor6o6.setVelocity(1)
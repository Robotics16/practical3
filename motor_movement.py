import brickpi
import time
import motor_util
import math

interface=brickpi.Interface()
interface.startLogging('logfile.txt')
interface.initialize()

motors = [0,3]
speed = 5

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])


motorParams = interface.MotorAngleControllerParameters()
motor_util.set_pid(motorParams)


interface.setMotorAngleControllerParameters(motors[0],motorParams)
interface.setMotorAngleControllerParameters(motors[1],motorParams)

print "Press Ctrl+C to exit"
while True:
        angle = float(input("Enter angle to rotate robot by (degrees)"))

        motor_util.rotate(angle*(math.pi/180), interface, motors)

        distance = float(input("Enter distance to move robot by (cm): "))

        motor_util.forward(distance, interface, motors)


interface.stopLogging('logfile.txt')
interface.terminate()


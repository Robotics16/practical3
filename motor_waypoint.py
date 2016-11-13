import brickpi
import time
import motor_util
import sys
import random
import math
import motor_square as ms

startPos = 0.0

def main():  
    interface = brickpi.Interface()
    #interface.startLogging('logfile.txt')
    interface.initialize()

    # Setup motors
    motors = [0,3]
    speed = 5
    interface.motorEnable(motors[0])
    interface.motorEnable(motors[1])
    motorParams = interface.MotorAngleControllerParameters()
    motor_util.set_pid(motorParams)
    interface.setMotorAngleControllerParameters(motors[0], motorParams)
    interface.setMotorAngleControllerParameters(motors[1], motorParams)

    # Setup initial state of particles
    numberOfParticles = 100
    particles = [((startPos, startPos, 0), 1 / float(numberOfParticles)) for i in range(numberOfParticles)]

    # Waypoint navigation:
    ms.drawParticles(particles)
    while (True):
        w_x = float(input("Enter your desired Wx position: "))
        w_y = float(input("Enter your desired Wy position: "))
        ms.navigateToWaypoint(w_x + startPos, w_y + startPos, particles, interface,               motors)
        ms.drawParticles(particles)

    print "Destination reached!"

    #interface.stopLogging('logfile.txt')
    interface.terminate()


if __name__ == "__main__":
    main()

import motor_particles
import motor_util
import time

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

navigateToWaypoint(84, 30, particles, interface, motors)
navigateToWaypoint(180, 30, particles, interface, motors)
navigateToWaypoint(180, 54, particles, interface, motors)
navigateToWaypoint(138, 54, particles, interface, motors)
navigateToWaypoint(138, 168, particles, interface, motors)
navigateToWaypoint(114, 168, particles, interface, motors)
navigateToWaypoint(114, 84, particles, interface, motors)
navigateToWaypoint(84, 84, particles, interface, motors)
navigateToWaypoint(84, 30, particles, interface, motors)

if __name__ == "__main__":
    main()

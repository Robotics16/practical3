import brickpi
import time
import motor_util
import sys
import random
import math
import numpy as np
from particleDataStructuresExample import mymap, canvas

startPos = 0.0

def drawParticles(particles):
    # offset = 300   # Offset used to draw on a sensible location on the screen
    # print "drawParticles:" + str([(x + offset, y + offset, th, w) for ((x, y, th), w) in particles])
    canvas.drawParticles([(x, y, th, w) for ((x, y, th), w) in particles])

def updateParticlesForward(particles, d):
    """Update all particles with d cm forward motion,
    based on current theta for each."""

    for i in range(0, len(particles)):
        updatedParticle = updateOneParticleForward(particles[i], d)
        particles[i] = updatedParticle

    return particles

def updateParticlesRotate(particles, angle):
    """Update all particles with a certain angle"""

    for i in range(0, len(particles)):
        updatedParticle = updateOneParticleRotate(particles[i], angle)
        particles[i] = updatedParticle

    return particles

def updateOneParticleForward(particle, d):
    """Update particle triple with d cm forward motion,
    use gaussian distribution with mu = 0 and estimated sigma"""
    ((old_x, old_y, old_theta), w) = particle

    sigma_offset   = 0.02            # estimated sigma for 10 cms (scaled with variance for longer distances) TODO
    sigma_rotation = 0.1  #0.100    # about 15 degrees for 114 cms
    sigma_rotation_scaled = sigma_rotation #math.sqrt((sigma_rotation ** 2) * d / 114)
    mu = 0

    e = random.gauss(mu, math.sqrt((sigma_offset ** 2) * d / 10)) # error term for coordinate offset
    f = random.gauss(mu, sigma_rotation_scaled) # error term for rotation

    particle = ((old_x + (d + e) * math.cos(old_theta),
                 old_y + (d + e) * math.sin(old_theta),
                 old_theta + f),
                w)

    return particle

def updateOneParticleRotate(particle, angle):
    """Update particle triple with angle radians rotation,
    use gaussian distribution with mu=0 and estimated sigma"""
    ((old_x, old_y, old_theta), w) = particle

    sigma_rotation_90 = 0.05  # estimated sigma for 90 degrees
    mu = 0

    sigma_rotation = sigma_rotation_90 #math.sqrt((sigma_rotation_90 ** 2) * abs(angle) / (math.pi / 2.0)) # scaled sigma based on the estimation and the actual angle
    g = random.gauss(mu, sigma_rotation) # error term for pure rotation

    particle = ((old_x, old_y, old_theta + angle + g), w)
    return particle


def getCurrentLocation(particles):
    """Given all particles returns an estimate of the
    current position (x, y, theta)"""
    estimates = [(x * weight, y * weight, theta * weight) for ((x, y, theta), weight) in particles]
    
    total_weight = sum([weight for ((x, y, theta), weight) in particles])
    
    x_estimate     = sum([e[0] for e in estimates]) / total_weight
    y_estimate     = sum([e[1] for e in estimates]) / total_weight
    theta_estimate = sum([e[2] for e in estimates]) / total_weight

    return (x_estimate, y_estimate, theta_estimate)

def navigateToWaypoint(w_x, w_y, particles, interface, motors):
    """Using the current location returned by getCurrentLocation()
    navigates the robot to (w_x, w_y) (coordinates in the Wold coordinate system)"""
    (x, y, theta) = getCurrentLocation(particles)

    # Get vector between current and next position
    (d_x, d_y) = (w_x - x, w_y - y)

    # Turn on the spot in the direction of the waypoint
    alpha = math.atan2(d_y, d_x) # absolute orientation needed (using atan2 so that the result is between -pi and pi)
    beta  = (alpha - theta) # angle to turn
    if (abs(beta) > math.pi) :
        if (beta > 0) :
            beta = beta - 2 * math.pi
        else :
            beta = beta + 2 * math.pi

    motor_util.rotate(-beta, interface, motors)
    particles = updateParticlesRotate(particles, beta)
    drawParticles(particles)
    print "Waypoint navigate: Rotate: " + str(beta)

    sonar_value = getSonarValue(interface)
    particles = updateparticleswithsonar(particles, sonar_value, mymap)
    drawParticles(particles)


    # Move straight forward to waypoint
    distance_to_move = math.sqrt(d_x ** 2 + d_y ** 2) # distance to move using the Pythagorean theorem
    print "Waypoint navigate: go: " + str(distance_to_move)
    motor_util.forward(distance_to_move, interface, motors)
    particles = updateParticlesForward(particles, distance_to_move)
    drawParticles(particles)

    # take sonar measurements (take 5 get median)
    sonar_value = getSonarValue(interface)

    # update probabilities with sonar distance
    particles = updateparticleswithsonar(particles, sonar_value, mymap)
    drawParticles(particles)

    return particles


def getSonarValue(interface):
    port = 3 # port which ultrasoic sensor is plugged in to
    interface.sensorEnable(port, brickpi.SensorType.SENSOR_ULTRASONIC)
    usReading = interface.getSensorValue(port)

    data_list = []
    length = 10

    while (length > 0):
        if usReading :
            data_list.append(usReading[0])
        #    print usReading
        else:
            print "Failed US reading"

        length = length - 1


    data_list.sort()
    sonar_value = (data_list[4] + data_list[5]) / 2

    print sonar_value
    
    return sonar_value


def updateparticleswithsonar(particles, measured_distance, map_geometry):
    """Updates all particle weights given the sonar measurement.
    Also normalizes and resamples the particle set"""

    for i in range(0, len(particles)):
        updatedParticle = updateOneParticleWithSonar(particles[i], measured_distance, map_geometry)
        particles[i] = updatedParticle

    particles = normalize_particles(particles)
    particles = resample_particles(particles)

    return particles

def updateOneParticleWithSonar(particle, measured_distance, map_geometry):
    """Updates a single particle weight based on the sonar measurement"""
    (pos, w) = particle
    
    if (measured_distance > 100):
        return particle
    
    w = w * calculate_likelihood(pos, measured_distance, map_geometry)
    particle = (pos, w)
    return particle

def calculate_likelihood(position, measured_distance, map_geometry):
    """Returns the probability of measuring `measured_distance` given the current
    position in the map"""

    (x, y, theta) = position
    walls = map_geometry.walls
    distances_wall = []

    for wall in walls:
        (a_x, a_y, b_x, b_y) = wall
        m = ((b_y - a_y) * (a_x - x) - (b_x - a_x) * (a_y - y)) / ((b_y - a_y) * math.cos(theta) - (b_x - a_x) * math.sin(theta))
        (intersection_x, intersection_y) = (x + m*math.cos(theta), y + m*math.sin(theta))

        is_between_x = (a_x <= intersection_x and intersection_x <= b_x) or (b_x <= intersection_x and intersection_x <= a_x)
        is_between_y = (a_y <= intersection_y and intersection_y <= b_y) or (b_y <= intersection_y and intersection_y <= a_y)

        if (is_between_x and is_between_y):
            distances_wall.append(m)


    greater_than_0_distances = [x for x in distances_wall if x >= 0]

    if not greater_than_0_distances:
        return 0

    minimum_distance = min(greater_than_0_distances)
    sigma = 0.3 # sonar error
    k = 0.001 # const to "lift" bell curve with

    dis_err = (measured_distance-minimum_distance)**2
    probability = math.exp(-dis_err/(2*sigma**2)) + k # p(z|m) + k

    return probability

def normalize_particles(particles):
    """Normalizes particle set, such that the weights of all particles add up to 1"""
    total_weight = sum([w for (pos, w) in particles])
    particles = [(pos, w / total_weight) for (pos, w) in particles]
    return particles

def resample_particles(particles):
    """Resamples normalized particle set: samples len(particles) number of particles from
    the provided particle set, where the chance of picking a particle during a sampling is the
    weight of the particle"""

    pdf = [w for (p,w) in particles]
    indices = list(np.random.choice(len(particles), len(particles), p=pdf))

    out = []

    for i in indices:
        out.append(particles[i])

    particles = out
    return out

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
    waypoint1_x = 84
    waypoint1_y = 30
    particles = [((waypoint1_x, waypoint1_y, 0), 1 / float(numberOfParticles)) for i in range(numberOfParticles)]

    drawParticles(particles)

    # Go in squares

    # for i in range(4):
    #     for j in range(4):
    #         motor_util.forward(10, interface, motors)
    #         drawParticles(particles)
    #         time.sleep(0.25)
    #         particles = updateParticlesForward(particles, 20)
    #         drawParticles(particles)
    #         time.sleep(0.25)
    #     motor_util.rotateRight90deg(interface, motors)
    #     particles = updateParticlesRotate(particles, -math.pi/2)
    #     drawParticles(particles)
    #     time.sleep(0.25)

    # print "Destination reached!"

    # Assume robot at waypoint 0 : (84, 30)
    particles = navigateToWaypoint(180, 30, particles, interface, motors)  # 1
    particles = navigateToWaypoint(180, 54, particles, interface, motors)  # 2
    particles = navigateToWaypoint(138, 54, particles, interface, motors)  # 3
    particles = navigateToWaypoint(138, 168, particles, interface, motors) # 4
    particles = navigateToWaypoint(114, 168, particles, interface, motors) # 5
    particles = navigateToWaypoint(114, 84, particles, interface, motors)  # 6
    particles = navigateToWaypoint(84, 84, particles, interface, motors)   # 7
    particles = navigateToWaypoint(84, 30, particles, interface, motors)   # 8

    #interface.stopLogging('logfile.txt')
    interface.terminate()


if __name__ == "__main__":
    main()

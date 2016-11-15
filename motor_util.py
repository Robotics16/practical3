import brickpi
import time
import math

rotate1deg = 6.54 / (math.pi/2)
forward1cm = 15.1 / 40

def rotate(angle, interface, motors):
  motorRot = angle*rotate1deg
  interface.increaseMotorAngleReferences(motors,[-motorRot,motorRot])

  while not interface.motorAngleReferencesReached(motors) :
          motorAngles = interface.getMotorAngles(motors)
          if motorAngles :
                #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
                time.sleep(0.1)

  print "Destination reached!"

def move(angle, interface, motors):
  interface.increaseMotorAngleReferences(motors,[angle,angle])

  while not interface.motorAngleReferencesReached(motors) :
          motorAngles = interface.getMotorAngles(motors)
          if motorAngles :
                #print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
                time.sleep(0.1)

  print "Destination reached!"

def set_pid(motorParams):
	motorParams.maxRotationAcceleration = 6.0
	motorParams.maxRotationSpeed = 12.0
	motorParams.feedForwardGain = 255/20.0
	motorParams.minPWM = 30.0
	motorParams.pidParameters.minOutput = -255
	motorParams.pidParameters.maxOutput = 255
	motorParams.pidParameters.k_p = 0.6 * 400.0 # 600
	motorParams.pidParameters.k_i = 200 #200 #700   # 200
	motorParams.pidParameters.k_d = 100 #0 #20.625

def rotateLeft90deg(interface, motors):
	rotate(math.pi/2, interface, motors)

def rotateRight90deg(interface, motors):
	rotate(-math.pi/2, interface, motors)

def forward(distance, interface, motors):
	move(distance*forward1cm, interface, motors)


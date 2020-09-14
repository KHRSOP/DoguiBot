"""seguidor_linha controller"""
from controller import Robot
from controller import Camera
import cv2
import numpy as np
import math
import time
robot = Robot()
timestep = int(robot.getBasicTimeStep())

#Camera

camera = Camera("camera")
camera.enable(timestep)
MAX_MOTORS = 16
L1 = 9    # UPPER_LEG_LENGTH [cm]
L2 = 8.5  # LOWER_LEG_LENGTH [cm]
M_PI = 3.14159265358979323846
gait_type = 5
i = 0
key = -1
backward = True
slf = 1
slf_min = 0
slf_max = 1
stride_length_factor = [slf, slf, slf, slf]
freq_min = 0.4
freq_max = 2
freq = 1.5
freq_offset = 0.2
ta_factor = [0, 0, 0, 0]
ta_min = -0.6
ta_max = 0.6
ta_offset = 0.6
gait_phase_shift = [
  [0, 0.5, 0, 0.5],      # trot
  [0, 0.5, 0.25, 0.75],  # walk
  [0, 0.1, 0.6, 0.5],    # gallop  (transverse)
  [0, 0.3, 0, 0.7],      # canter
  [0, 0.5, 0.5, 0],      # pace
  [0, 0, 0.5, 0.5],      # bound
  [0, 0, 0, 0]           # pronk
]
display_info = False
nome_motor = ["pelvis",
              "front_left_1",
              "front_right_1",
              "front_left_2",
              "front_right_2",
              "front_left_3",
              "front_right_3",
              "back_left_1",
              "back_right_1",
              "back_left_2",
              "back_right_2",
              "back_left_3",
              "back_right_3",
              "neck_1",
              "neck_2",
              "head"]   
              
PELVIS = 0
FRONT_LEFT_1 = 1
FRONT_RIGHT_1 = 2
FRONT_LEFT_2 = 3
FRONT_RIGHT_2 = 4
FRONT_LEFT_3 = 5
FRONT_RIGHT_3 = 6
BACK_LEFT_1 = 7
BACK_RIGHT_1 = 8
BACK_LEFT_2 = 9
BACK_RIGHT_2 = 10 
BACK_LEFT_3 = 11
BACK_RIGHT_3 = 12
NECK_1 = 13
NECK_2 = 14
HEAD = 15
              
gait_setup = [[BACK_RIGHT_1, BACK_RIGHT_3],
             [BACK_LEFT_1, BACK_LEFT_3],
             [FRONT_LEFT_1, FRONT_LEFT_3],
             [FRONT_RIGHT_1, FRONT_RIGHT_3]]    
             
             
def computeWalkingPosition(motorsPosition, t, gait_freq, gait_type, legId,
                           stride_length_factor, backward):
  global L1, L2, gait_phase_shift
  # proceed to reverse kinematic to compute leg position
  freq = gait_freq
  # compute modulo
  n = (int)(t / (1 / freq))
  t = t - n * (1 / freq)
  # reverse time sequence for backward walk
  if backward:
    t = (1 / freq) - t
  # ellipsoid parameters
  a = 0.95 * L1 * stride_length_factor
  h = 0
  k = -(L1 + L2 / 2)
  b = -k - math.sqrt(L1 * L1 + L2 * L2)
  # compute ellipsoid points
  x = h + a * math.cos(2 * M_PI * freq * t + gait_phase_shift[gait_type][legId] * 2 * M_PI)
  y = k + b * math.sin(2 * M_PI * freq * t + gait_phase_shift[gait_type][legId] * 2 * M_PI)
  # compute angle A2
  A2 = math.acos((x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2))
  # compute angle A1
  A1 = math.acos(((L1 + L2 * math.cos(A2)) * x - (-L2 * math.sin(A2)) * y) / (math.pow(L1 + L2 * math.cos(A2), 2) + math.pow(-L2 * math.sin(A2), 2)));
  # subtract 2PI
  A1 = M_PI / 2 - A1
  motorsPosition[0] = A1 * -1
  motorsPosition[1] = A2 * -1
  
  print(motorsPosition[0])
  print(motorsPosition[1])
_stepCount = 0
stand = 0
robot.getMotor('neck_1').setPosition(-M_PI / 2)
robot.getMotor('front_left_2').setPosition(M_PI / 2)
robot.getMotor('front_right_2').setPosition(-M_PI / 2)
robot.getMotor('back_left_2').setPosition(M_PI / 2)
robot.getMotor('back_right_2').setPosition(-M_PI / 2)
print("LEVANTOU")

# Main loop:
while robot.step(timestep) != -1:
    motorPositions = [0, 0]
    for legId in range(4):
        #time.sleep(0.1)
        computeWalkingPosition(motorPositions, _stepCount * (timestep / 1000), freq, gait_type, legId, stride_length_factor[legId], backward)
        robot.getMotor(nome_motor[gait_setup[legId][1]]).setPosition(motorPositions[1])
        robot.getMotor(nome_motor[gait_setup[legId][0]]).setPosition(motorPositions[0])
        #print("COMECA A PRINTAR")
    _stepCount = _stepCount + 1

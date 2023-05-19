import math
import numpy as np
from pyquaternion import Quaternion

BODY_JOINTS =['pelvis','femur_r', 'tibia_r', 'talus_r', 'calcn_r', 'toes_r',
              'femur_l', 'tibia_l', 'talus_l', 'calcn_l', 'toes_l',
              'torso', 
              'humerus_r', 'ulna_r', 'radius_r', 'hand_r', 
              'humerus_l', 'ulna_l', 'radius_l', 'hand_l']


BODY_JOINTS_IN_DP_ORDER = ['pelvis','femur_r', 'tibia_r', 'talus_r', 'calcn_r', 'toes_r',
                         'femur_l', 'tibia_l', 'talus_l', 'calcn_l', 'toes_l',
                        'torso', 
                        'humerus_r', 'ulna_r', 'radius_r', 'hand_r', 
                        'humerus_l', 'ulna_l', 'radius_l', 'hand_l']


DOF_DEF = { 'pelvis':3,'femur_r':3,'tibia_r':1,'talus_r':1,'calcn_r':1,'toes_r':1,
            'femur_l':3,'tibia_l':1,'talus_l':1,'calcn_l':1,'toes_l':1,
            'torso':3,
            'humerus_r':3,'ulna_r':1,'radius_r':1,'hand_r':1,
            'humerus_l':3,'ulna_l':1,'radius_l':1,'hand_l':1,}

BODY_DEFS = ['femur_r', 'tibia_r', 'talus_r', 'calcn_r', 'toes_r',
             'femur_l', 'tibia_l', 'talus_l', 'calcn_l', 'toes_l',
             'torso', 
             'humerus_r', 'ulna_r', 'radius_r', 'hand_r', 
             'humerus_l', 'ulna_l', 'radius_l', 'hand_l']

# PARAMS_KP_KD = {"torso": [1000, 100], "humerus_r": [400, 40], "ulna_r": [300, 30], 
#         "humerus_l": [400, 40], "ulna_l": [300, 30], "femur_r": [500, 50], "tibia_r": [500, 50], 
#         "talus_r": [400, 40], "calcn_r": [400, 40], "femur_l": [500, 50], "tibia_l": [500, 50], "talus_l": [400, 40], "calcn_l": [400, 40]}

PARAMS_KP_KD = {'pelvis':[1000, 100], 
             'femur_r': [500,50], 'tibia_r':[500,50], 'talus_r':[400,40], 'calcn_r':[400,40], 'toes_r':[200,20],
             'femur_l':[500,50], 'tibia_l':[500,50], 'talus_l':[400,40], 'calcn_l':[400,40], 'toes_l':[20,20],
             'torso':[1000, 100], 
             'humerus_r':[400, 40], 'ulna_r':[300,30], 'radius_r':[300,30], 'hand_r':[100,10], 
             'humerus_l':[400, 40], 'ulna_l':[300,30], 'radius_l':[300,30], 'hand_l':[100,10]}

# JOINT_WEIGHT = {"pelvis": 1, "torso": 0.5, "femur_r": 0.5, 
#                 "tibia_r": 0.3, "calcn_r": 0.2, "humerus_r": 0.3, "ulna_r": 0.2, 
#                  "femur_l": 0.5, "tibia_l": 0.3, "talus_l": 0.2, "calcn_l":0.2,
#                 "humerus_l": 0.3, "ulna_l": 0.2}

JOINT_WEIGHT = {'pelvis': 1, 'femur_r': 0.5, 'tibia_r':0.3, 'talus_r':0.2, 'calcn_r':0.2, 'toes_r':0.01,
             'femur_l':0.5, 'tibia_l':0.3, 'talus_l':0.2, 'calcn_l':0.2, 'toes_l':0.01,
             'torso':1, 
             'humerus_r':0.5, 'ulna_r':0.2, 'radius_r':0.2, 'hand_r':0.01, 
             'humerus_l':0.5, 'ulna_l':0.2, 'radius_l':0.2, 'hand_l':0.01}

def align_rotation(rot):
    q_input = Quaternion(rot[0], rot[1], rot[2], rot[3])
    q_align_right = Quaternion(matrix=np.array([[1.0, 0.0, 0.0], 
                                                [0.0, 0.0, 1.0], 
                                                [0.0, -1.0, 0.0]]))
    q_align_left = Quaternion(matrix=np.array([[1.0, 0.0, 0.0], 
                                               [0.0, 0.0, -1.0], 
                                               [0.0, 1.0, 0.0]]))
    q_output = q_align_left * q_input * q_align_right
    output = q_output.elements
    return [output[0], output[2], output[1], output[3]]

def align_position(pos):
    assert len(pos) == 3
    left_matrix = np.array([[1.0, 0.0, 0.0], 
                            [0.0, 0.0, -1.0], 
                            [0.0, 1.0, 0.0]])
    pos_output = np.matmul(left_matrix, pos)
    return pos_output

def calc_angular_vel_from_quaternion(orien_0, orien_1, dt):
    seg0 = align_rotation(orien_0)
    seg1 = align_rotation(orien_1)

    q_0 = Quaternion(seg0[0], seg0[1], seg0[2], seg0[3])
    q_1 = Quaternion(seg1[0], seg1[1], seg1[2], seg1[3])

    q_diff =  q_0.conjugate * q_1
    # q_diff =  q_1 * q_0.conjugate
    axis = q_diff.axis
    angle = q_diff.angle
    
    tmp_vel = (angle * 1.0)/dt * axis
    vel_angular = [tmp_vel[0], tmp_vel[1], tmp_vel[2]]

    return vel_angular

def calc_diff_from_quaternion(orien_0, orien_1):
    seg0 = align_rotation(orien_0)
    seg1 = align_rotation(orien_1)

    q_0 = Quaternion(seg0[0], seg0[1], seg0[2], seg0[3])
    q_1 = Quaternion(seg1[0], seg1[1], seg1[2], seg1[3])

    q_diff =  q_0.conjugate * q_1
    # q_diff =  q_1 * q_0.conjugate
    angle = q_diff.angle
    return angle
# 
# def xyzrot2quat(xyzrot):
#     sx, cx = math.sin(xyzrot[0]), math.cos(xyzrot[0])
#     sy, cy = math.sin(xyzrot[1]), math.cos(xyzrot[1])
#     sz, cz = math.sin(xyzrot[2]), math.cos(xyzrot[2])
# 
#     R_x = np.array([[1.0, 0.0, 0.0],
#                     [0.0, cx, -sx],
#                     [0.0, sx, cx]])
# 
#     R_y = np.array([[cy, 0.0, sy],
#                     [0.0, 1, 0.0],
#                     [-sy, 0.0, cy]])
# 
#     R_z = np.array([[cz, -sz, 0.0],
#                     [sz, cz, 0.0],
#                     [0.0, 0.0, 1.0]])
#     
#     rot = np.matmul( np.matmul(R_x, R_y), R_z)
# 
#     quat = Quaternion(matrix=rot)
# 
#     return  quat.elements
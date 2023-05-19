from mujoco_py import MjSim, load_model_from_path, MjViewer
import numpy as np
from pyquaternion import Quaternion

import json

human = "helper/subject_scaled_run_converted/subject_scaled_run_converted.xml"
humanoid = "helper/subject_scaled_run_converted/orig_humanoid.xml"

human_model = load_model_from_path(human)
hoid_model = load_model_from_path(humanoid)

sim_humanoid = MjSim(hoid_model)
sim_human = MjSim(human_model)


data_human = sim_human.data
data_hoid = sim_humanoid.data

in_mocap = [1,2,3,7,8] # [1,2,3,7,8,13,14,17,19]
zeros = 0.0
frame2 = []
# joint_order = ['pelvis', 'femur_r', 'tibia_r', 'talus_r', 'calcn_r', 'toes_r',
#                          'femur_l', 'tibia_l', 'talus_l', 'calcn_l', 'toes_l',
#                         'torso', 
#                         'humerus_r', 'ulna_r', 'radius_r', 'hand_r', 
#                         'humerus_l', 'ulna_l', 'radius_l', 'hand_l']


with open("helper/motions/humanoid3d_walk.txt", 'r') as fin:
    data = json.load(fin)
    
    motions = np.array(data["Frames"])
    # print(motions.shape)
    for x in range(len(motions)):
        temp = []
        dur = motions[x][0]
        root_pos = motions[x][1:4].tolist()
        root_rot = motions[x][4:8].tolist()
        chest_rot = motions[x][8:12].tolist()
        neck_rot = motions[x][12:16].tolist()
        r_hip = motions[x][16:20].tolist()
        r_knee = motions[x][20]
        r_ank = motions[x][21:25].tolist()
        r_shol = motions[x][25:29].tolist()
        r_el = motions[x][29]
        l_hip = motions[x][30:34].tolist()
        l_knee = motions[x][34]
        l_ank = motions[x][35:39].tolist()
        l_shol = motions[x][39:43].tolist()
        l_el = motions[x][43]

        bodies = [root_rot,r_hip,r_knee,l_hip,l_knee,r_shol,r_el,l_shol,l_el]

        
        temp.append(dur)
        for i in range(1,human_model.nbody):
            if i not in in_mocap:
                if human_model.body_dofnum[i] == 1:
                        temp.append(zeros)
                else:
                    for i in range(4):
                        temp.append(zeros)

            else:
                if isinstance(bodies[in_mocap.index(i)], float):
                    temp.append(bodies[in_mocap.index(i)])
                else:
                    for val in bodies[in_mocap.index(i)]:
                        temp.append(val)
            
        frame2.append(temp)


targetFile = 'helper/motions/humanoid3d_testLeg_scaled.txt'

with open(targetFile, 'w') as f:
    f.write('{"Loop": "wrap"\n, "Frames": [ \n')

    for x in frame2[:-1]:
        f.write(f'{x},\n')
    f.write(f'{frame2[-1]}\n')


    f.write(']}')


        




        
        
        



 


        

    




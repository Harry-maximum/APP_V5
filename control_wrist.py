# Copyright 2024 Fourier Intelligence
# Author: Yizhao Shi <yizhao.shi@fftai.com>

from __future__ import annotations


import time
from robot import Robot
from fi_fsa import fi_fsa_v1
import numpy as np
from helper_fcn import string_input_hdlr
import math
from rotation_sampler import RotationSampler, RotationSampleListener 



class control_wrist:
    def __init__(self, Robot, frequency):
        self.Robot = Robot
        self.frequency = frequency
        self.EE_rotyaw_step = 0.01
        self.last_q = self.Robot.q_arms()
        self.listener =RotationSampleListener()
        self.sampler = RotationSampler(1/100, self.listener)
        self.rotation_sampler_update = np.zeros(2)
    def control_wrist_yaw(self, rot_input):

        # rot_input_array = string_input_hdlr(rot_input)
        EE_num = rot_input[0]
        rot_step = rot_input[1]
        rot_jump_dir = rot_input[2]

        upper_ee_q = self.Robot.q_arms()

        # last_q = upper_ee_q
        
        if rot_step == 0 : 
            if EE_num == 1:
                if rot_jump_dir == -1: # arrow left: left hand rotate con_clk_w
                    if upper_ee_q[4] < 1.84 and upper_ee_q[4] >= 0.27:
                        upper_ee_q[4] = 1.84
                    elif upper_ee_q[4] < 0.27 and upper_ee_q[4] >= -1.3:
                        upper_ee_q[4] = 0.27
                    elif upper_ee_q[4] < -1.3:
                        upper_ee_q[4] = -1.3
            
                elif rot_jump_dir == 1: # arrow right: left hand rotate clk_w
                    if upper_ee_q[4] > -1.3 and upper_ee_q[4] <= 0.27:
                        upper_ee_q[4] = -1.3
                    elif upper_ee_q[4] > 0.27 and upper_ee_q[4] <= 1.84:
                        upper_ee_q[4] = 0.27
                    elif upper_ee_q[4] > 1.84:
                        upper_ee_q[4] = 1.84
            
            elif EE_num ==2:

                if rot_jump_dir == -1: # X: right hand rotate con_clk_w
                    if upper_ee_q[11] < 1.3 and upper_ee_q[11] >= -0.27:
                        upper_ee_q[11] = 1.3
                    elif upper_ee_q[11] < -0.27 and upper_ee_q[11] >= -1.84:
                        upper_ee_q[11] = -0.27
                    elif upper_ee_q[11] < -1.84:
                        upper_ee_q[11] = -1.84
                
                elif rot_jump_dir == 1: # B: right hand rotate clk_w
                    if upper_ee_q[11] > -1.84 and upper_ee_q[11] <= -0.27:
                        upper_ee_q[11] = -1.84
                    elif upper_ee_q[11] > -0.27 and upper_ee_q[11] <= 1.3:
                        upper_ee_q[11] = -0.27
                    elif upper_ee_q[11] > 1.3:
                        upper_ee_q[11] = 1.3

            self.sampler.update_data_source('HAND', 'ROTATION', upper_ee_q[4], upper_ee_q[11])
            self.rotation_sampler_update = self.listener.on_sample_update('HAND', 'ROTATION', upper_ee_q[4], upper_ee_q[11])

            upper_ee_q[4] = self.rotation_sampler_update[0]

            upper_ee_q[11] = self.rotation_sampler_update[1]
            
            # time.sleep(1)
            self.last_q = upper_ee_q
            print(upper_ee_q)

            self.Robot.set_joints(self.Robot.LIST_upper_joint, upper_ee_q)
            # self.Robot.viz.display(self.Robot.configuration.q)
            self.Robot.rate.sleep()

            ##### robot experiment required ######
       
            for i in range(len(self.Robot.upper_actuator_IP_list)):
                
                if self.Robot.upper_actuator_IP_list[i][-2:] == "10" or self.Robot.upper_actuator_IP_list[i][-2:] == "11" \
                    or self.Robot.upper_actuator_IP_list[i][-2:] == "13" or self.Robot.upper_actuator_IP_list[i][-2:] == "31" or self.Robot.upper_actuator_IP_list[i][-2:] == "36":
                    fi_fsa_v1.fast_set_position_control(self.Robot.upper_actuator_IP_list[i], -upper_ee_q[i] * 180/math.pi)
                else:
                    fi_fsa_v1.fast_set_position_control(self.Robot.upper_actuator_IP_list[i], upper_ee_q[i] * 180/math.pi)
                    
                    
            time.sleep(1/self.frequency)

            #####################################


        elif rot_step != 0:
            
            if EE_num ==1:
                
                upper_ee_q_update = upper_ee_q.copy()
                #check button input to move left_ee along z axis, move only 1 button pressed
                upper_ee_q_update[4] -= self.EE_rotyaw_step * rot_step 
                # left_stay = False
                upper_ee_q = upper_ee_q_update
                
                # last_q = upper_ee_q
                # print("start q", last_q)

                if self.Robot.WARNING_joint_limit(upper_ee_q):
                    upper_ee_q = self.last_q
            
            if EE_num ==2:

            
                upper_ee_q_update = upper_ee_q.copy()
                #check button input to move right_ee along z axis, move only 1 button pressed
                upper_ee_q_update[11] -= self.EE_rotyaw_step * rot_step

                upper_ee_q = upper_ee_q_update

                if self.Robot.WARNING_joint_limit(upper_ee_q):
                    upper_ee_q = self.last_q

            self.sampler.update_data_source('HAND', 'ROTATION', upper_ee_q[4], upper_ee_q[11])
            self.rotation_sampler_update = self.listener.on_sample_update('HAND', 'ROTATION', upper_ee_q[4], upper_ee_q[11])

            upper_ee_q[4] = self.rotation_sampler_update[0]

            upper_ee_q[11] = self.rotation_sampler_update[1]


            self.Robot.set_joints(self.Robot.LIST_upper_joint, upper_ee_q)
            # self.Robot.viz.display(self.Robot.configuration.q)
            self.Robot.rate.sleep()

            ##### robot experiment required ######
          
            for i in range(len(self.Robot.upper_actuator_IP_list)):
                
                if self.Robot.upper_actuator_IP_list[i][-2:] == "10" or self.Robot.upper_actuator_IP_list[i][-2:] == "11" \
                    or self.Robot.upper_actuator_IP_list[i][-2:] == "13" or self.Robot.upper_actuator_IP_list[i][-2:] == "31" or self.Robot.upper_actuator_IP_list[i][-2:] == "36":
                    fi_fsa_v1.fast_set_position_control(self.Robot.upper_actuator_IP_list[i], -upper_ee_q[i] * 180/math.pi)
                else:
                    fi_fsa_v1.fast_set_position_control(self.Robot.upper_actuator_IP_list[i], upper_ee_q[i] * 180/math.pi)
                             
            time.sleep(1/self.frequency)
            #####################################

# Copyright 2024 Fourier Intelligence
# Author: Yizhao Shi <yizhao.shi@fftai.com>

from __future__ import annotations


import numpy as np
import time
from fourier_grx.sdk import ControlGroup, RobotClient
from scipy.spatial.transform import Rotation as R
from fi_fsa import fi_fsa_v1
import math
import helper_fcn
from fi_fsa import fi_fsa_v1
from calculation import movel_scipy, move_arc_yaw
from robot import Robot
from motion_sampler import MotionSampler, MotionSampleListener

class control_position:
    def __init__(self, Robot, frequency):

        self.Robot = Robot
        self.frequency = frequency
        self.POS_left_shoulder_pitch_joint = self.Robot.get_transform("left_shoulder_roll_joint", "base_link")
        self.POS_right_shoulder_pitch_joint = self.Robot.get_transform("right_shoulder_roll_joint", "base_link")


        self.listener = MotionSampleListener()
        self.sampler = MotionSampler(1/300, self.listener)

        self.motion_sampler_update = np.zeros(6)

        self.EE_moveX_step = 0.05
        self.EE_moveY_step = 0.01
        self.EE_moveZup_step = 0.01
        self.EE_moveZdown_step = 0.01

    def movel_left_right(self, second_right, second_left, target_position_left_input = None, target_position_right_input = None):
        
        joints_list = []

        collision_WARNING = False
        limit_WARNING = False

        now_position_left = self.Robot.get_transform("left_ee_frame", "base_link")
        now_position_right = self.Robot.get_transform("right_ee_frame", "base_link")
        
        len_left = 0
        len_right = 0


        if target_position_left_input is not None:
            position_list_left, rotation_list_left = movel_scipy(now_position_left, target_position_left_input, 2)
            # self.Robot.frequency*second_left)#movel_list
            len_left = len(position_list_left)
            
        
        if target_position_right_input is not None:
            position_list_right, rotation_list_right = movel_scipy(now_position_right, target_position_right_input, 2)
            # self.Robot.frequency*second_right)#movel_list
            len_right = len(position_list_right)

        
        joints_list.append((self.Robot.q_arms()).tolist())

        for i in range(max(len_right, len_left)):
            point2_right = None
            point2_left = None
            if i < len_right:
                point1_right = np.concatenate((rotation_list_right[i][0],np.array([position_list_right[i]]).T), axis=1)
                point2_right = np.vstack((point1_right, np.array([[0,0,0,1]])))
            if i < len_left:
                point1_left = np.concatenate((rotation_list_left[i][0],np.array([position_list_left[i]]).T), axis=1)
                point2_left = np.vstack((point1_left, np.array([[0,0,0,1]])))

            collision_WARNING, limit_WARNING = self.Robot.inverse_kinematics(point2_left, point2_right)

            print(collision_WARNING, limit_WARNING)

            if collision_WARNING == False and limit_WARNING == False:
                joints_list.append(self.Robot.q_arms().tolist())


            self.Robot.rate.sleep()

        return joints_list # results in radian


    def control_EE_position(self, move_input):

        # print("another loop")
        
        # move_input_array = helper_fcn.string_input_hdlr(move_input)
        # print("move_input_array", move_input_array)
        # EE_num = move_input_array[0]
        x_left_input = move_input[0]
        y_left_input = move_input[1]
        z_left_input = move_input[2]
        x_right_input = move_input[3]
        y_right_input = move_input[4]
        z_right_input = move_input[5]

        left_ee_pos = self.Robot.get_transform("left_ee_frame", "base_link")
        right_ee_pos = self.Robot.get_transform("right_ee_frame", "base_link")

        left_ee_pos_update = None
        right_ee_pos_update = None


        #left arm control

        # if EE_num == 1:
        if x_left_input != 0 or y_left_input != 0:


            left_ee_pos_update = left_ee_pos.copy()
        
            left_rot_center = np.transpose(np.hstack((self.POS_left_shoulder_pitch_joint[:2, 3],left_ee_pos[3,3])))

            left_ee_pos_update = move_arc_yaw( left_ee_pos, left_rot_center, -x_left_input * 0.01 * self.EE_moveX_step)

            left_ee_pos_update[:2,3] -= y_left_input * 0.01 * self.EE_moveY_step * left_ee_pos_update[:2,2]

            ## check button input to move left_ee along z axis, move only 1 button pressed
        
            if z_left_input == 1:
                left_ee_pos_update[2,3] += self.EE_moveZup_step * z_left_input  
            else:
                left_ee_pos_update[2,3] += self.EE_moveZdown_step * z_left_input * 0.01

        
            left_ee_pos = left_ee_pos_update

        
        elif z_left_input != 0:

            left_ee_pos_update = left_ee_pos.copy()
            #check button input to move left_ee along z axis, move only 1 button pressed
            if z_left_input == 1:
                left_ee_pos_update[2,3] += self.EE_moveZup_step * z_left_input 
            else:
                left_ee_pos_update[2,3] += self.EE_moveZdown_step * z_left_input * 0.01

            left_ee_pos = left_ee_pos_update

        # if EE_num == 2:

        # right arm control
        if x_right_input != 0 or y_right_input != 0:

            right_ee_pos_update = right_ee_pos.copy()

            right_rot_center = np.transpose(np.hstack((self.POS_right_shoulder_pitch_joint[:2, 3], right_ee_pos[3,3])))

            right_ee_pos_update = move_arc_yaw(right_ee_pos, right_rot_center, -x_right_input * 0.01 * self.EE_moveX_step)

            right_ee_pos_update[:2,3] -= y_right_input * 0.01 * self.EE_moveY_step * right_ee_pos_update[:2,2]
            
            #check button input to move right_ee along z axis, move only 1 button pressed
            if z_right_input == 1:
                right_ee_pos_update[2,3] += self.EE_moveZup_step * z_right_input  
            else:
                right_ee_pos_update[2,3] += self.EE_moveZdown_step * z_right_input * 0.01

            right_ee_pos = right_ee_pos_update

        
        elif z_right_input != 0:
            right_ee_pos_update = right_ee_pos.copy()
            #check button input to move right_ee along z axis, move only 1 button pressed
            if z_right_input == 1:
                right_ee_pos_update[2,3] += self.EE_moveZup_step * z_right_input  
            else:
                right_ee_pos_update[2,3] += self.EE_moveZdown_step * z_right_input * 0.01
            right_ee_pos = right_ee_pos_update

        
        self.sampler.update_data_source('HAND', 'MOVE', left_ee_pos[0,3], left_ee_pos[1,3], left_ee_pos[2,3], right_ee_pos[0,3], right_ee_pos[1,3], right_ee_pos[2,3])

        self.motion_sampler_update = self.listener.on_sample_update('HAND', 'MOVE', left_ee_pos[0,3], left_ee_pos[1,3], left_ee_pos[2,3], right_ee_pos[0,3], right_ee_pos[1,3], right_ee_pos[2,3])

        left_ee_pos[:3, 3] = np.transpose(self.motion_sampler_update[:3])

        right_ee_pos[:3, 3] = np.transpose(self.motion_sampler_update[-3:])
        
        joints_move_list = self.movel_left_right(0.02, 0.02, left_ee_pos, right_ee_pos)

        
        ###### robot experiment required ######
        joints_move_array = np.array(joints_move_list)
      
        count = 0

        while count < len(joints_move_list):

            for i in range(len(self.Robot.upper_actuator_IP_list)):
                    
                if self.Robot.upper_actuator_IP_list[i][-2:] == "10" or self.Robot.upper_actuator_IP_list[i][-2:] == "11" \
                    or self.Robot.upper_actuator_IP_list[i][-2:] == "13" or self.Robot.upper_actuator_IP_list[i][-2:] == "31" or self.Robot.upper_actuator_IP_list[i][-2:] == "36":
                    fi_fsa_v1.fast_set_position_control(self.Robot.upper_actuator_IP_list[i], -joints_move_array[count, i] * 180/math.pi)
                else:
                    fi_fsa_v1.fast_set_position_control(self.Robot.upper_actuator_IP_list[i], joints_move_array[count, i] * 180/math.pi)
            
            time.sleep(1/self.frequency)
            count += 1
        ####################################
        

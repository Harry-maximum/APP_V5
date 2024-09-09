# Copyright 2024 Fourier Intelligence
# Author: Yizhao Shi <yizhao.shi@fftai.com>

from __future__ import annotations

from fourier_grx.sdk import ControlGroup
from trajectory_planning import polynomial_trejectory_planing
from fi_fsa import fi_fsa_v1
import numpy as np
import math
import time


def startup(Robot, frequency):

    LIST_arm_target = Robot.LIST_upper_joint

    # Q_left_current = Robot.q_arms()[-14:-7]
    # Q_right_current =  Robot.q_arms()[-7:]

    # left = [0, 0.3, 0, -1.57, 0.27, 0, 0]
    # right = [0, 0.3, 0, -1.57, 0.27, 0, 0]

    left_waypoints_pos = np.array([[0, 0, 0, 0, 0, 0, 0], [30, 20, 30, -70, -20, 0, 0], [-30, 10, 10, -80, -20, 10, 0], [0, 18, 0, -90, 15, 0, 0] ])/180*math.pi
    right_waypoints_pos = np.array([[0, 0, 0, 0, 0, 0, 0], [30, -20, -30, -70, 20, 0, 0], [-30, -10, -10, -80, 20, -10, 0], [0, -18, 0, -90, -15, 0, 0] ])/180*math.pi

    time_stamps =[0, 2, 4, 6]
    

    left_trajectory_position, left_trajectory_velocity = polynomial_trejectory_planing(left_waypoints_pos, time_stamps)
    right_trajectory_position, right_trajectory_velocity = polynomial_trejectory_planing(right_waypoints_pos, time_stamps)

    arm_trajectory_position = np.hstack((left_trajectory_position, right_trajectory_position))
    
    # create matrix store velocity info for all joints
    arm_trajectory_velocity_copy = np.zeros((len(left_trajectory_velocity),39))

 
    arm_trajectory_velocity_copy[:,-16:-9] = left_trajectory_velocity

  
    arm_trajectory_velocity_copy[:,-8:-1] = right_trajectory_velocity

    count=0
  
    while count < len(arm_trajectory_position):
        
        Robot.set_joints(LIST_arm_target,arm_trajectory_position[count])
        Robot.configuration.integrate_inplace(arm_trajectory_velocity_copy[count], Robot.dt)
        # Robot.viz.display(Robot.configuration.q)
   
    
        fsa_traj_position = arm_trajectory_position.copy()

        for i in range(len(Robot.upper_actuator_IP_list)):
                
                if Robot.upper_actuator_IP_list[i][-2:] == "10" or Robot.upper_actuator_IP_list[i][-2:] == "11" \
                    or Robot.upper_actuator_IP_list[i][-2:] == "13" or Robot.upper_actuator_IP_list[i][-2:] == "31" or Robot.upper_actuator_IP_list[i][-2:] == "36":
                    fi_fsa_v1.fast_set_position_control(Robot.upper_actuator_IP_list[i], -arm_trajectory_position[count, i] * 180/math.pi)
                else:
                    fi_fsa_v1.fast_set_position_control(Robot.upper_actuator_IP_list[i], arm_trajectory_position[count, i] * 180/math.pi)
                
                
        
        time.sleep(1/frequency)
      
        Robot.rate.sleep()

        count+=1

    left_ee_pos = Robot.get_transform("left_ee_frame", "base_link")
    right_ee_pos = Robot.get_transform("right_ee_frame", "base_link")


def move_to_init(Robot, client, frequency, set_left = False, set_right = False):

    LIST_arm_target = Robot.LIST_upper_joint

    Q_left_current = Robot.q_arms()[-14:-7]
    Q_right_current =  Robot.q_arms()[-7:]


    time_stamps =[0, 2]

    left_waypoints_pos = np.array([Q_left_current,Q_left_current])
    right_waypoints_pos = np.array([Q_right_current,Q_right_current])

    
    if set_left == True:
    
        left_waypoints_pos = np.array([Q_left_current, Robot.q_left_init])

        
    if set_right == True:
    
        right_waypoints_pos = np.array([Q_right_current, Robot.q_right_init])


    left_trajectory_position, left_trajectory_velocity = polynomial_trejectory_planing(left_waypoints_pos, time_stamps)
    right_trajectory_position, right_trajectory_velocity = polynomial_trejectory_planing(right_waypoints_pos, time_stamps)

    arm_trajectory_position = np.hstack((left_trajectory_position, right_trajectory_position))
    
    # create matrix store velocity info for all joints
    arm_trajectory_velocity_copy = np.zeros((len(left_trajectory_velocity),39))

    if set_left == True:
        arm_trajectory_velocity_copy[:,-16:-9] = left_trajectory_velocity

    if set_right == True:
        arm_trajectory_velocity_copy[:,-8:-1] = right_trajectory_velocity

    count=0

    while count < len(arm_trajectory_position):
        

        Robot.set_joints(LIST_arm_target,arm_trajectory_position[count])
        Robot.configuration.integrate_inplace(arm_trajectory_velocity_copy[count], Robot.dt)
        # Robot.viz.display(Robot.configuration.q)
    
    
        ###### robot experiment required ###### 
        
        for i in range(len(Robot.upper_actuator_IP_list)):
                
            if Robot.upper_actuator_IP_list[i][-2:] == "10" or Robot.upper_actuator_IP_list[i][-2:] == "11" \
                or Robot.upper_actuator_IP_list[i][-2:] == "13" or Robot.upper_actuator_IP_list[i][-2:] == "31" or Robot.upper_actuator_IP_list[i][-2:] == "36":
                fi_fsa_v1.fast_set_position_control(Robot.upper_actuator_IP_list[i], -arm_trajectory_position[count, i] * 180/math.pi)
            else:
                fi_fsa_v1.fast_set_position_control(Robot.upper_actuator_IP_list[i], arm_trajectory_position[count, i] * 180/math.pi)
            
                
        time.sleep(1/frequency)
        ########################################

        Robot.rate.sleep()

        count+=1
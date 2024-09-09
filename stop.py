import os
import numpy as np
import pinocchio as pin
import qpsolvers
from loop_rate_limiters import RateLimiter
import math
from rich import inspect
from trajectory_planning import polynomial_trejectory_planing
import time
from fi_fsa import fi_fsa_v1
from robot import Robot


def move_to_zero(Robot, client):

    LIST_arm_target = Robot.LIST_upper_joint
    Q_left_current = Robot.q_arms()[-14:-7]
    Q_right_current =  Robot.q_arms()[-7:]

    Q_zero=[0, 0, 0, 0, 0, 0, 0] 
    time_stamps =[0, 2]

    left_waypoints_pos = np.array([Q_left_current, Q_zero])
    right_waypoints_pos = np.array([Q_right_current, Q_zero])

    left_trajectory_position, left_trajectory_velocity = polynomial_trejectory_planing(left_waypoints_pos, time_stamps)
    right_trajectory_position, right_trajectory_velocity = polynomial_trejectory_planing(right_waypoints_pos, time_stamps)

    arm_trajectory_position = np.hstack((left_trajectory_position, right_trajectory_position))
    
    # create matrix store velocity info for all joints
    arm_trajectory_velocity_copy = np.zeros((len(left_trajectory_velocity),39))

    arm_trajectory_velocity_copy[:,-16:-9] = left_trajectory_velocity

    arm_trajectory_velocity_copy[:,-8:-1] = right_trajectory_velocity

    count=0

    Robot.left_gripper.close()
    Robot.right_gripper.close()

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
                
        time.sleep(1/100)

        ########################################

        Robot.rate.sleep()

        count+=1

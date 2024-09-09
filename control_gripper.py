# Copyright 2024 Fourier Intelligence
# Author: Yizhao Shi <yizhao.shi@fftai.com>

from __future__ import annotations
from fi_fsa import fi_fsa_v1


def flip_gripper(Robot, gripper_num: str):

    jaw_q = Robot.q_jaws()

    print("jaw_q[0]", jaw_q[0])
    print("jaw_q[1]", jaw_q[1])

    if gripper_num == "1":
        if jaw_q[0] < 0.3:

            Robot.set_joint("left_jaw_joint", 0.9)
           
            fi_fsa_v1.fast_set_position_control(Robot.left_gripper_IP, 60)
            # fsa_state = fi_fsa_v1.get_state(Robot.left_gripper_IP)
            # print("State = %d" % fsa_state)
            
        elif jaw_q[0] > 0.7:

            Robot.set_joint("left_jaw_joint", 0.1)
           
            fi_fsa_v1.fast_set_position_control(Robot.left_gripper_IP, 10)
            # fsa_state = fi_fsa_v1.get_state(Robot.left_gripper_IP)
            # print("State = %d" % fsa_state)
            
    elif gripper_num == "2":
        if jaw_q[1] > -0.3:

            Robot.set_joint("right_jaw_joint", -0.9)
        
            fi_fsa_v1.fast_set_position_control(Robot.right_gripper_IP, 60)
            # fsa_state = fi_fsa_v1.get_state(Robot.right_gripper_IP)
            # print("State = %d" % fsa_state)
            
        elif jaw_q[1] < -0.7:

            Robot.set_joint("right_jaw_joint", -0.1)
          
            fi_fsa_v1.fast_set_position_control(Robot.right_gripper_IP, 10)
            # fsa_state = fi_fsa_v1.get_state(Robot.left_gripper_IP)
            # print("State = %d" % fsa_state)
        
    # Robot.viz.display(Robot.configuration.q)

   
    
    


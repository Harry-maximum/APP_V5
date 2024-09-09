# Copyright 2024 Fourier Intelligence
# Author: Yizhao Shi <yizhao.shi@fftai.com>

from __future__ import annotations

import numpy as np
import sys
import os
from robot import Robot
import time
from init import move_to_init, startup 
from control_gripper import flip_gripper
from control_wrist import control_wrist
from control_position import control_position
from stop import move_to_zero
from fi_fsa import fi_fsa_v1
from zenoh_comm import zenoh_comm 
from helper_fcn import string_input_hdlr
from start_with_gr1.mode_listener import mode_listener
import subprocess
from multiprocessing import Process, Queue

def producer(queue, upper_mode):
    queue.put(upper_mode)

# def consumer(queue):
#     return queue.get()

def main():


   # 创建并启动线程
    frequency = 150

    client = 0

    print("client started")
    # robot initialization
   
    remote_control_start = False

    gr1 = Robot("T2", "../GR1T2_jaw/urdf/robot.urdf", control_frequency = frequency)

    for i in range(len(gr1.upper_actuator_IP_list)):
            fi_fsa_v1.set_position_control(gr1.upper_actuator_IP_list[i], 0.0)
            fi_fsa_v1.set_enable(gr1.upper_actuator_IP_list[i])
            fi_fsa_v1.set_mode_of_operation(
                gr1.upper_actuator_IP_list[i], fi_fsa_v1.FSAModeOfOperation.POSITION_CONTROL
            )

    fi_fsa_v1.set_position_control(gr1.left_gripper_IP, 0.0)
    fi_fsa_v1.set_enable(gr1.left_gripper_IP)
    fi_fsa_v1.set_mode_of_operation(gr1.left_gripper_IP, fi_fsa_v1.FSAModeOfOperation.POSITION_CONTROL)

    fi_fsa_v1.set_position_control(gr1.right_gripper_IP, 0.0)
    fi_fsa_v1.set_enable(gr1.right_gripper_IP)
    fi_fsa_v1.set_mode_of_operation(gr1.right_gripper_IP, fi_fsa_v1.FSAModeOfOperation.POSITION_CONTROL)


    time.sleep(0.2)

    z_comm = zenoh_comm(frequency)
    # gr1_mode =  mode_listener()

    queue = Queue()
    # pdcr = Process(target = producer, args = (queue,))
    # csmr = Process(target = consumer, args = (queue,))

    # pdcr.start()
    # csmr.start()


    rot_wrist = control_wrist(gr1, frequency)
    move_ee = control_position(gr1, frequency)

    init_input = z_comm.init_input 
    stop_input = z_comm.stop_input    
    grab_input = z_comm.grab_input 
    move_input = z_comm.move_input 
    rot_input = z_comm.rot_input

    upper_op_ongoing = '0'
 
    print("waiting for init...")

    remote_control_start = True
 
    startup(gr1, frequency)


    while remote_control_start:
        # print("i am in")

        init_input = z_comm.init_input 
        stop_input = z_comm.stop_input
        grab_input = z_comm.grab_input 
        move_input = z_comm.move_input 
        rot_input = z_comm.rot_input 
        
        # upper_op_ongoing =  gr1_mode.upper_control_mode

        if type(move_input) is not type(None):
            move_input = string_input_hdlr(move_input)
        if type(rot_input) is not type(None):
            rot_input = string_input_hdlr(rot_input)
        # print("move_input_array", move_input_array)

        # print("move_input", move_input)
        # print(type(move_input))

        # print("rot_input", rot_input)
        # print(type(rot_input))
        

        if stop_input is not None:
            print("stop!!!!")
            move_to_zero(gr1, client)
            remote_control_start = False
            z_comm.stop_complete = True
            # upper_op_ongoing = '0'
            # producer(queue, upper_op_ongoing)

            # gr1_mode.quit_all()
            # print("gr1_mode stop",gr1_mode.__hash__)
                       # print(gr1_mode.upper_control_mode)
            time.sleep(1)
            result = subprocess.Popen(["/home/gr1p24ap0058/remote_control_upper/APP_V5/start_with_gr1/run_grx_upper.sh", "stop_algorithm"])
            # result = subprocess.Popen(["/home/gr1p24ap0058/remote_control_upper/APP_V5/start_with_gr1/run_grx_upper.sh", "stop"])
            
            break
        

        if init_input is not None:
            if init_input == "0":
                move_to_init(gr1, client, frequency, set_left = True, set_right = True)
            elif init_input == "1":
                move_to_init(gr1, client, frequency, set_left = True, set_right = False)
            elif init_input == "2":
                move_to_init(gr1, client, frequency, set_left = False, set_right = True)
            z_comm.init_complete = True

            time.sleep(1)
            
            continue


        if grab_input is not None:
            flip_gripper(gr1, grab_input)
            z_comm.grab_complete = True
    

        if move_input is not None and any(move_input != 0):
                
            move_ee.control_EE_position(move_input)

    
        if rot_input is not None and (rot_input[1] != 0 or rot_input[2] != 0):
            
            rot_wrist.control_wrist_yaw(rot_input)


        time.sleep(1/frequency)




if __name__=="__main__":

    main()
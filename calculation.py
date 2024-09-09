# Copyright 2024 Fourier Intelligence
# Author: Yizhao Shi <yizhao.shi@fftai.com>

from __future__ import annotations


import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from rich import inspect
from scipy.spatial.transform import Slerp



def move_arc_yaw(now_position, center_pos_input, move_angle):
    """
    calculate the target pos of end effactor(EE) in the base frame after rotate a step angle

    :param now_position: current RT pose of end effector in the base frame
    :center_pos_input: pos of the joint input for calculating virtual center of rotation
    :move_angle: unit rotation anble (in radian)
    :return: the target RT pose in base frame
    """
    start_pos = now_position[:3, 3]
    # start_rot = now_position[:3, :3]

    virtual_center_pos = center_pos_input.copy()
    virtual_center_pos[2] = start_pos[2]
    
    print("virtual_center_pos")
    print(virtual_center_pos)

    H_vf_O = np.eye(4)
    H_vf_O[:3, 3] = virtual_center_pos

    H_cur_EE_O = now_position

    rotation_matrix = R.from_euler('z', move_angle).as_matrix()
    rotation_transform = np.eye(4)
    rotation_transform[:3, :3] = rotation_matrix

    # print(rotation_transform)

    H_cur_EE_O_update = H_vf_O @ rotation_transform @ np.linalg.inv(H_vf_O) @ H_cur_EE_O

    # print("target RT")
    # print(H_left_EE_O_update)

    return H_cur_EE_O_update

def move_arc_pitch(now_position, center_pos_input, move_angle):
    """
    calculate the target pos of end effactor(EE) in the base frame after rotate a step angle

    :param now_position: current RT pose of end effector in the base frame
    :center_pos_input: pos of the joint input for calculating virtual center of rotation
    :move_angle: unit rotation anble (in radian)
    :return: the target RT pose in base frame
    """
    start_pos = now_position[:3, 3]
    # start_rot = now_position[:3, :3]

    virtual_center_pos = center_pos_input.copy()
    virtual_center_pos[1] = start_pos[1]

    H_vf_O = np.eye(4)
    H_vf_O[:3, 3] = virtual_center_pos

    H_cur_EE_O = now_position

    rotation_matrix = R.from_euler('y', -move_angle).as_matrix()
    rotation_transform = np.eye(4)
    rotation_transform[:3, :3] = rotation_matrix

    H_cur_EE_O_update = H_vf_O @ rotation_transform @ np.linalg.inv(H_vf_O) @ H_cur_EE_O


    return H_cur_EE_O_update

def movel_scipy(now_position, target_position, n):

    rots = R.from_matrix([now_position[:3, :3],target_position[:3, :3]])
    
    rot1 = R.from_matrix(now_position[:3, :3])
    rot2 = R.from_matrix(target_position[:3, :3])

    slerp = Slerp([0,1], rots)

    rot = R.from_matrix(np.dot(np.linalg.inv(now_position[:3,:3]), target_position[:3,:3] ))
    # print(rot1)
    inspect(rot1)
    rotation_list = []
    position_list = []
    rotation_seq = [rot1,rot2]
  
    for i in range(int(n)):
        interpolated_rot = slerp([i/n])
        R_interpolated = interpolated_rot.as_matrix()

        rotation_list.append(R_interpolated)
        position_list.append([now_position[0][3]+(target_position[0][3]-now_position[0][3])*i/n, now_position[1][3]+(target_position[1][3]-now_position[1][3])*i/n, now_position[2][3]+(target_position[2][3]-now_position[2][3])*i/n])
    return position_list, rotation_list
# Copyright 2024 Fourier Intelligence
# Author: Yizhao Shi <yizhao.shi@fftai.com>

from __future__ import annotations

import os
import numpy as np
import pink
import pinocchio as pin
import qpsolvers
from loop_rate_limiters import RateLimiter
from scipy.spatial.transform import Rotation as R
import math
from rich import inspect
from trajectory_planning import polynomial_trejectory_planing
from scipy.spatial.transform import Slerp
import sys
import time
from fourier_grx.sdk import ControlGroup, RobotClient
from fourier_grx.sdk.end_effectors import Gripper

class Robot:
    def __init__(self, robot_name: str, urdf_path: str, control_frequency: int, joint_names: list[str] = None, visualize=False):
        if joint_names is None:
            joint_names = []
        self.robot_name = robot_name
        self.robot = pin.RobotWrapper.BuildFromURDF(
            filename=urdf_path,
            package_dirs=["../GR1T2_jaw/meshes"],
            root_joint=pin.JointModelFreeFlyer(),
        )
        inspect(self.robot.model)

        if joint_names:
            for j in joint_names:
                assert self.robot.model.existJointName(j), f"Joint {j} does not exist."
            self._joint_names = joint_names
        else:
            self._joint_names = list(self.robot.model.names)
            self._joint_names.remove("universe")
            self._joint_names.remove("root_joint")

        assert self.robot.nq == 32 + 9, "nq mismatch."

        self.robot = self.robot.buildReducedRobot(["waist_roll_joint"])
        self.movel_joints_list = []        

        self.right_ee_frame = self.robot.model.addFrame(
        pin.Frame(
            "right_ee_frame",
            self.robot.model.getJointId("right_wrist_pitch_joint"),
            self.robot.model.getFrameId("right_hand_pitch_link"),
            # self.robot.model.getJointId("right_jaw_joint"),
            # self.robot.model.getFrameId("right_jaw_link"),
            pin.SE3(
                rotation=np.eye(3),
                translation=np.array([0.0, 0.0, 0.0]),
            ),
            pin.FrameType.OP_FRAME,
        )
    )

        self.left_ee_frame = self.robot.model.addFrame(
            pin.Frame(
                "left_ee_frame",
                self.robot.model.getJointId("left_wrist_pitch_joint"),
                self.robot.model.getFrameId("left_hand_pitch_link"),
                # self.robot.model.getJointId("left_jaw_joint"),
                # self.robot.model.getFrameId("left_jaw_link"),
                pin.SE3(
                    rotation=np.eye(3),
                    translation=np.array([0.0, 0.0, 0.0]),
                ),
                pin.FrameType.OP_FRAME,
            )
        )


        self.robot.rebuildData()
        self.configuration = pink.Configuration(self.robot.model, self.robot.data, self.robot.q0, copy_data=False)
        self.viz = None
        if visualize:
            self.viz = pin.visualize.MeshcatVisualizer(
                self.robot.model, self.robot.collision_model, self.robot.visual_model
            )
            # import ipdb;
            # ipdb.set_trace()
            self.robot.setVisualizer(self.viz, init=False)
            self.viz.initViewer(open=True)
            self.viz.loadViewerModel()
            self.viz.displayFrames(True,(self.robot.model.getFrameId("left_ee_frame"),self.robot.model.getFrameId("base_link"),self.robot.model.getFrameId("torso_link"),self.robot.model.getFrameId("right_ee_frame")))
            

        self.geom_model = pin.buildGeomFromUrdf(self.robot.model, urdf_path, pin.GeometryType.COLLISION, "../GR1T2_jaw/meshes")

        self.collision1 = pin.CollisionPair(self.geom_model.getGeometryId ("left_hand_yaw_link_0"), self.geom_model.getGeometryId ("right_hand_yaw_link_0"))
        self.collision2 = pin.CollisionPair(self.geom_model.getGeometryId ("left_upper_arm_yaw_link_0"), self.geom_model.getGeometryId ("right_hand_yaw_link_0"))
        self.collision2 = pin.CollisionPair(self.geom_model.getGeometryId ("right_upper_arm_yaw_link_0"), self.geom_model.getGeometryId ("left_hand_yaw_link_0"))
        self.collision3 = pin.CollisionPair(self.geom_model.getGeometryId ("left_upper_arm_yaw_link_0"), self.geom_model.getGeometryId ("torso_link_0"))
        self.collision4 = pin.CollisionPair(self.geom_model.getGeometryId ("right_upper_arm_yaw_link_0"), self.geom_model.getGeometryId ("torso_link_0"))

        self.collision5 = pin.CollisionPair(self.geom_model.getGeometryId ("left_hand_yaw_link_0"), self.geom_model.getGeometryId ("right_jaw_link_0"))
        self.collision6 = pin.CollisionPair(self.geom_model.getGeometryId ("right_hand_yaw_link_0"), self.geom_model.getGeometryId ("left_jaw_link_0"))
        self.collision7 = pin.CollisionPair(self.geom_model.getGeometryId ("right_jaw_link_0"), self.geom_model.getGeometryId ("left_jaw_link_0"))
        self.collision8 = pin.CollisionPair(self.geom_model.getGeometryId ("left_jaw_link_0"), self.geom_model.getGeometryId ("torso_link_0"))
        self.collision9 = pin.CollisionPair(self.geom_model.getGeometryId ("right_jaw_link_0"), self.geom_model.getGeometryId ("torso_link_0"))


        self.geom_model.addCollisionPair(self.collision1)
        self.geom_model.addCollisionPair(self.collision2)
        self.geom_model.addCollisionPair(self.collision3)
        self.geom_model.addCollisionPair(self.collision4)
        self.geom_model.addCollisionPair(self.collision5)
        self.geom_model.addCollisionPair(self.collision6)
        self.geom_model.addCollisionPair(self.collision7)
        self.geom_model.addCollisionPair(self.collision8)
        self.geom_model.addCollisionPair(self.collision9)


        self.geom_data = pin.GeometryData(self.geom_model)

        self.upper_actuator_IP_list = ["192.168.137.10", "192.168.137.11", "192.168.137.12", "192.168.137.13", "192.168.137.14", "192.168.137.15", "192.168.137.16",
                                       "192.168.137.30", "192.168.137.31", "192.168.137.32", "192.168.137.33", "192.168.137.34", "192.168.137.35", "192.168.137.36"]

        self.left_gripper_IP = "192.168.137.17"
        self.right_gripper_IP = "192.168.137.37"

        self.LIST_left_joint= ['left_shoulder_pitch_joint','left_shoulder_roll_joint','left_shoulder_yaw_joint', 'left_elbow_pitch_joint', 'left_wrist_yaw_joint','left_wrist_roll_joint', 'left_wrist_pitch_joint']
        self.LIST_right_joint= ['right_shoulder_pitch_joint','right_shoulder_roll_joint','right_shoulder_yaw_joint', 'right_elbow_pitch_joint', 'right_wrist_yaw_joint','right_wrist_roll_joint','right_wrist_pitch_joint']
        self.LIST_upper_joint = self.LIST_left_joint + self.LIST_right_joint

        self.q_left_init = [0, 0.3, 0, -1.57, 0.27, 0, 0]
        self.q_right_init = [0, -0.3, 0, -1.57, -0.27, 0, 0]

        self.LIMIT_upper_joints_top = np.array([1.92,3.27,2.97,2.27,2.97,0.61,0.87,1.92,0.57,2.97,2.27,2.97,0.96,0.61])
        self.LIMIT_upper_joints_bottom = np.array([-2.79,-0.57,-2.97,-2.27,-2.97,-0.61,-0.96, -2.79,-3.27,-2.97,-2.27,-2.97,-0.87,-0.61])
        
        self.left_gripper = Gripper("left")
        self.right_gripper = Gripper("right")
        
        self.right_hand_task = pink.tasks.RelativeFrameTask(
            "right_ee_frame",
            "torso_link",
            position_cost=0.95,
            orientation_cost=[0.5, 0.5, 0.5],
            gain=0.95,
            )
    
        self.left_hand_task = pink.tasks.RelativeFrameTask(
            "left_ee_frame",
            "torso_link",
            position_cost=0.95,
            orientation_cost=[0.5, 0.5, 0.5],
            gain=0.95,
        )
        self.tasks = [self.right_hand_task, self.left_hand_task]
        self.solver = qpsolvers.available_solvers[0]
        if "quadprog" in qpsolvers.available_solvers:
            self.solver = "quadprog"

        self.frequency = control_frequency
        self.rate = RateLimiter(frequency=self.frequency)
        self.dt = self.rate.period
        self.configuration.update(self.robot.q0)


        for i in range(len(self.robot.model.joints)):
            # self.robot.model.velocityLimit[i]/=2
            if self.robot.model.velocityLimit[i]>=37.3/2:
                self.robot.model.velocityLimit[i]=37.3/2
        

    @property
    def joint_names(self):
        return self._joint_names

    @property
    def frame_names(self):
        return [f.name for f in self.robot.model.frames]

    def get_idx_q(self, joint_name: str):
        idx = self.robot.model.getJointId(joint_name)
        
        return self.robot.model.idx_qs[idx]

    def set_joints(self, joint_names: list[str], q: np.ndarray):
        q_new = self.configuration.q.copy()
        for j, qj in zip(joint_names, q, strict=False):
            idx = self.get_idx_q(j)
            q_new[idx] = qj
        self.configuration.update(q_new)

    def set_joint(self, joint_name: str, q_1d: float):
        idx = self.get_idx_q(joint_name)
        q_new = self.configuration.q.copy()
        q_new[idx] = q_1d
        self.configuration.update(q_new)

    def q_arms(self):
        q = self.configuration.q[-16:-9].copy()
        q = np.hstack((q, self.configuration.q[-8:-1].copy()))
        return q

    def q_jaws(self):
        q = self.configuration.q[-9].copy()
        q = np.hstack((q, self.configuration.q[-1].copy()))
        return q
    

    @property
    def q(self):
        return self.configuration.q[7:].copy()
    
    @q.setter
    def q(self, q: np.ndarray):
        self.configuration.update(np.hstack((np.zeros(7), q)))

    @property
    def q_dict(self):
        return dict(zip(self.joint_names, self.q, strict=False))

    @q_dict.setter
    def q_dict(self, q_dict: dict):
        self.set_joints(q_dict.keys(), q_dict.values())

    def get_transform(self, to_frame: str, from_frame: str):
        """Get the pose of a frame with respect to another frame.

        Args:
            to_frame: Name of the frame to get the pose of.
            from_frame: Name of the frame to get the pose in.

        Returns:
            Current transform from the source frame to the dest frame.

        Raises:
            KeyError: if any of the frame names is not found in the model.
        """
        if from_frame not in self.frame_names:
            raise KeyError(f"Frame {from_frame} not found in the robot.")
        if to_frame not in self.frame_names:
            raise KeyError(f"Frame {to_frame} not found in the robot.")

        transform = self.configuration.get_transform(to_frame, from_frame).np
        return transform

    def update(self):
        if self.viz: 
            self.viz.display(self.configuration.q)
    
    def collision_check(self):
        collision_flag = pin.computeCollisions(self.robot.model, self.robot.data, self.geom_model, self.geom_data, self.configuration.q, False)
        print("this is the collision flag")
        print(collision_flag)
        return collision_flag
    
    def get_collision_distance(self):
        print("collision distance:")
        min_dis = 1000000
        pin.computeDistances(self.robot.model, self.robot.data, self.geom_model, self.geom_data, self.configuration.q)
        for i in range(len(self.geom_model.collisionPairs)):
            dr = self.geom_data.distanceResults[i].min_distance
            if dr<min_dis:
                min_dis = dr
        print(min_dis)
        return min_dis
    
    def WARNING_joint_limit(self, current_upper_q):
        # upper_q = self.configuration.q[-14:]
        # print(upper_q)
        upper_q = current_upper_q.copy()

        for i in range(len(self.LIST_upper_joint)):
            # print("upper limit", i)
            # print(self.robot.model.upperPositionLimit[self.robot.model.getJointId(self.LIST_upper_joint[i])])
            # print("lower limit", i)
            # print(self.robot.model.lowerPositionLimit[self.robot.model.getJointId(self.LIST_upper_joint[i])])
            # print(self.configuration.q[i])
            if self.LIMIT_upper_joints_top[i] <= upper_q[i]:
                print("joint high limit exceeded")
                print("limit:", self.LIMIT_upper_joints_top[i])
                print("current q:", upper_q[i])
                return True
            if self.LIMIT_upper_joints_bottom[i] >= upper_q[i]:
                print("joint lower limit exceeded")
                print("limit:", self.LIMIT_upper_joints_bottom[i])
                print("current q:", upper_q[i])
                return True
            
     





    def inverse_kinematics(self, TARGET_left_ee=None, TARGET_right_ee=None):

        ##### Funtion Description: IK with collision check and limit check

        # upper joints list
        
        

        WARNING_collision = False
        WARNING_limit = False
        last_q = self.q_arms()
        ERR_left = 0
        ERR_right = 0

        # now_position_right = Robot.get_transform("right_ee_frame", "base_link")
        # now_position_left = Robot.get_transform("left_ee_frame", "base_link")
        # target_right = Robot.get_transform("right_ee_frame", "base_link")
        # target_left = Robot.get_transform("left_ee_frame", "base_link")


        # print("original pos")
        # print(last_q)
        # print("original pos")
        
        
        if type(TARGET_left_ee)!= type(None):
            # type(TARGET_left_ee) 
            l_target_base_torso = np.dot(self.get_transform("base_link", "torso_link") , TARGET_left_ee) 
            l_target_base_torso_SE3 = pin.SE3(rotation=l_target_base_torso[:3, :3],
            translation=l_target_base_torso[:3, 3],)
            self.left_hand_task.set_target(l_target_base_torso_SE3)
        if type(TARGET_right_ee)!= type(None):
            r_target_base_torso = np.dot(self.get_transform("base_link", "torso_link") , TARGET_right_ee) 
            r_target_base_torso_SE3 = pin.SE3(rotation=r_target_base_torso[:3, :3],
            translation=r_target_base_torso[:3, 3],)
            self.right_hand_task.set_target(r_target_base_torso_SE3)
        

        if type(TARGET_left_ee)==type(None) and type(TARGET_right_ee)!=type(None):
            velocity = pink.solve_ik(self.configuration, [self.right_hand_task], self.dt, solver=self.solver)
        elif type(TARGET_left_ee)!=type(None) and type(TARGET_right_ee)==type(None):
            velocity = pink.solve_ik(self.configuration, [self.left_hand_task], self.dt, solver=self.solver)
        else:
            velocity = pink.solve_ik(self.configuration, self.tasks, self.dt, solver=self.solver)
        
        
        self.configuration.integrate_inplace(velocity, self.dt)

        POS_right_plan = self.get_transform("right_ee_frame", "base_link")
        POS_left_plan = self.get_transform("left_ee_frame", "base_link")

        # print("updated pos")
        # print(self.configuration.q)
        # print("updated pos")
        
        collision_distance = self.get_collision_distance()

        if type(TARGET_left_ee) != type(None):
            ERR_left = math.sqrt((POS_left_plan[0][3]-TARGET_left_ee[0][3])**2+(POS_left_plan[1][3]-TARGET_left_ee[1][3])**2+(POS_left_plan[2][3]-TARGET_left_ee[2][3])**2)
            print("ERR_left = ", ERR_left)

        elif type(TARGET_right_ee) != type(None):
            ERR_right = math.sqrt((POS_right_plan[0][3]-TARGET_right_ee[0][3])**2+(POS_right_plan[1][3]-TARGET_right_ee[1][3])**2+(POS_right_plan[2][3]-TARGET_right_ee[2][3])**2)
            print("ERR_right = ", ERR_right)

        # inspect(self.robot.model)

        if collision_distance <= 0.02:

            WARNING_collision = True

            # print("pos SHOULD BE UPDATED")
            # print(last_q)
            # print("pos SHOULD BE UPDATED")

            self.set_joints(self.LIST_upper_joint, last_q)

            # print(self.configuration.q)

            print("Collision Warning!!!")

            print("modified pos")
            print(self.configuration.q)
            print("modified pos")
        elif ERR_left >= 0.001 or ERR_right >= 0.001 or self.WARNING_joint_limit(self.q_arms()):

            WARNING_limit = True

            self.set_joints(self.LIST_upper_joint, last_q)

            print("Limit Warning!!!")


        # self.viz.display(self.configuration.q)
        # print("DISPLAYED pos")
        # print(self.configuration.q)
        # print("DISPLAYED pos")


        return WARNING_collision, WARNING_limit


    
    def inverse_kinematics_integral(self, TARGET_left_ee=None,TARGET_right_ee=None):
        
        if type(TARGET_left_ee) != type(None):
            l_target_base_torso = np.dot(self.get_transform("base_link", "torso_link") , TARGET_left_ee) 
            l_target_base_torso_SE3 = pin.SE3(rotation=l_target_base_torso[:3, :3], translation=l_target_base_torso[:3, 3],)
            self.left_hand_task.set_target(l_target_base_torso_SE3)
        
        if type(TARGET_right_ee) != type(None):        
            r_target_base_torso = np.dot(self.get_transform("base_link", "torso_link") , TARGET_right_ee) 
            r_target_base_torso_SE3 = pin.SE3(rotation=r_target_base_torso[:3, :3], translation=r_target_base_torso[:3, 3],)
            self.right_hand_task.set_target(r_target_base_torso_SE3)
        
        self.right_hand_task.set_target(r_target_base_torso_SE3)
        self.left_hand_task.set_target(l_target_base_torso_SE3)

        current_configuration_q = self.configuration.q
        while True:
            if type(TARGET_left_ee)==type(None) and type(TARGET_right_ee)!=type(None):
                velocity = pink.solve_ik(self.configuration, [self.right_hand_task], self.dt, solver=self.solver)
            elif type(TARGET_left_ee)!=type(None) and type(TARGET_right_ee)==type(None):
                velocity = pink.solve_ik(self.configuration, [self.left_hand_task], self.dt, solver=self.solver)
            else:
                velocity = pink.solve_ik(self.configuration, self.tasks, self.dt, solver=self.solver)
            self.configuration.integrate_inplace(velocity, self.dt)
            if np.linalg.norm(self.right_hand_task.compute_error(self.configuration)) < 0.005 and np.linalg.norm(self.left_hand_task.compute_error(self.configuration)) < 0.005:
                break
        update_configuration_q = self.configuration.q
        self.configuration.q = current_configuration_q
        return update_configuration_q
    

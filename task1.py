"""
        切换到task1 -> 开电灯 10分

        运动控制模式

        1、房间1墙面上有一个白色开关，标识有绿色提示灯的一端为“开”，当开关为“开”时，电灯状态为红色。当开关为“关”状态时，电灯为白色。
        2、本任务要求，Walker需按下开关的“开”（绿色标志一端），电灯打开亮红色持续3秒及以上。
        3、调用SwitchLight任务，Walker位于*1-开电灯房间的固定位置，开关与Walker的相对位置固定，Walker无需移动足部位置，可通过运动规划手臂完成任务。
        4、任务1的总运行时间不超过1分钟，逾时则任务挑战失败。
    Author: John
"""

import rospy
from utils.walker_scenes import scene_task1_SwitchLight
from utils.walker_tops_srvs import BuildConnect, WalkerWebotsSub
from utils import WxWebotsApi
from utils.walker_types import JointCommand
import math
import numpy as np
import time

def is_check_position_same(cur_position, check_position):
    cur_p = np.array(list(cur_position))
    check_p = np.array(list(check_position))
    res = abs(check_p - cur_p)
    if sum(res<=0.03) >= 7:
        return True
    else:
        return False


def task1():
    # 1.切换到特定的场景
    scene_task1_SwitchLight()
    time.sleep(5)
    # 2.1获取相应的控制器
    WxCotroApi = WxWebotsApi.WxWebotsControllers()
    WxJointApi = WxWebotsApi.WxWebotsJoints()
    ll_controller = WxCotroApi.publish_leftLimb_controller

    # 订阅信息
    ll = WxWebotsApi.WxWebotsJoints()
    ll.sub_leftLimb_joint_states()
    """
    LShoulderPitch -0.785 3.14     96.2
    LShoulderRoll  -1.57  0.0175   96.2
    LShoulderYaw   -1.92  1.92     96.2
    LElbowRoll     -2.27  1.00E-06 96.2
    LElbowYaw      -2.36  2.36     27.3
    LWristRoll     -0.351 0.351    27.3
    LWristPitch    -0.351 0.351    27.3

    RShoulderPitch -1.00E-06 1.5708   96.2
    RShoulderRoll  -3.14     0.785    96.2
    RShoulderYaw   -1.57     0.0175   96.2
    RElbowRoll     -1.92     1.92     96.2
    RElbowYaw      -2.27     1.00E-06 27.3
    9RWristRoll    -2.36     2.36     27.3
    RWristPitch    -0.351    0.351    27.3

    LShoulderYaw : RShoulderYaw 反
    LElbowYaw : RElbowYaw 反


    # LFinger -1.00E-06 1.5708 0.5
    # lh_command = [-0.0005440629177677493, -0.0014368724491023198, 
    # 0.0011593607539626544, 0.0034412821446439526, 
    # 0.001125063615987366, 0.003441281729176153, 
    # 0.002425551821042031, 0.005156504778905103, 
    # 0.001874385406552332, 0.0054440861704677206]

    """

    # 2.2 定义参数,参数格式可查看函数获取
    # time
    # time = 0.01
    # 左臂参数
    l_names = ['' for _ in range(7)]
    l_command = [0 for _ in range(7)]


    l_command_1 = [0,0,-0.6,0,0.8,0,-0.2]
    l_command_2 = [0,0,-1.3,0,1.6,0,-0.35]
    l_command_3 = [0,0,-1.8,-2.0,2.3,0,-0.35]
    l_command_4 = [0.4,-0.1,-1.8,-2.0,2.3,0,-0.35]
    l_command_5 = [0.9,-0.2,-1.8,-1.2,2.3,0,-0.35]
    is_status_1 = False
    is_status_2 = False
    is_status_3 = False
    is_status_4 = False
    is_status_5 = False
    status = 1

        # 合并
    l_commands = [l_command_1, l_command_2, l_command_3, l_command_4, l_command_5]

    # 定义标签
    is_l_over = False
    # 获取拟合数据
    l_commands = WxWebotsApi.line_fit(l_commands, timeline=1000000)
    while not rospy.is_shutdown():
        if not is_l_over :
            # 发布到topic
            for l_cmd in l_commands:
                ll_controller(l_names, l_cmd)
            is_l_over = True

    # 2.3 发布到相应的topic
    # while not rospy.is_shutdown():
    #     # l_command[2] = 1.85  # LShoulderPitch
    #     # l_command[1] = -0.7     # 肩 打开/关闭臂
    #     # l_command[3] = 1.12     # 肘 打开/关闭肘
    #     # l_command[0] = 1.1      # 肩 举起/放下臂
    #     # l_command[4] = -0.5     # 肘 旋转肘

    #     leftLimb_joint_states = ll.leftLimb_joint_states
    #     if leftLimb_joint_states:
    #         position = list(leftLimb_joint_states.position)
    #         if is_check_position_same(position, l_command_1) and not is_status_1:
    #             status = 2
    #             is_status_1 = True
    #         elif is_check_position_same(position, l_command_2) and not is_status_2:
    #             status = 3
    #             is_status_2 = True
    #         elif is_check_position_same(position, l_command_3) and not is_status_3:
    #             status = 4
    #             is_status_3 = True
    #         elif is_check_position_same(position, l_command_4) and not is_status_3:
    #             status = 5
    #             is_status_4 = True
        
    #     if status == 1:
    #         ll_controller(l_names, l_command_1)
    #     elif status == 2:
    #         ll_controller(l_names, l_command_2)
    #     elif status == 3:
    #         ll_controller(l_names, l_command_3)
    #     elif status == 4:
    #         ll_controller(l_names, l_command_4)
    #     elif status == 5:
    #         ll_controller(l_names, l_command_5)


        # l_command[2] = -0.785
        # l_command[4] = 2
        # l_command[1] = -1.45
        # l_command[3] = -1.65
        # l_command[0] = 1.25
        # ll_controller(l_names, l_command)  # 传入参数控制左臂
        
        # LeftLimb_joint_states = WxJointApi.leftLimb_joint_states  #订阅左臂信息
        # if LeftLimb_joint_states:
        #     print(LeftLimb_joint_states)
            
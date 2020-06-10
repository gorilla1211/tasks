"""
    Task6: Push Cart
        推平板车 30分, 运动控制模式

        1、客厅3放置有1个平板车，平板车前方地面两侧有2道红色引导线。
        2、本任务要求，Walker双手握住平板车把手，直线推动平板车，直至平板车越过红色引导线。行走过程中，平板车必须始终处于两道引导线中间范围。
        3、调用PushCart任务，Walker出现于客厅3的固定位置，平板车与Walker的相对位置固定，Walker无需移动足部位置即可接触到平板车把手，可通过运动规划手臂完成任务。
        4、任务6的总运行时间不超过3分钟，逾时则任务挑战失败。

    Author: Dongbox
"""
import rospy
from utils.walker_scenes import scene_task6_PushCart
from utils.walker_tops_srvs import BuildConnect, WalkerWebotsSub
from utils import WxWebotsApi
from utils.walker_types import JointCommand
import math
import matplotlib.pyplot as plt
import numpy as np
import time


def is_check_position_same(cur_position, check_position):
    cur_p = np.array(list(cur_position))
    check_p = np.array(list(check_position))
    res = abs(check_p - cur_p)
    if sum(res<=0.001) >= 6:
        return True
    else:
        return False


def task6():
    # 1.切换到对应场景
    # scene_task6_PushCart()
    # time
    # 2.1 获取所需的控制器
    WxContrApi = WxWebotsApi.WxWebotsControllers()
    ws = WxWebotsApi.WxWebotsStepFunc()
    rl_controller = WxContrApi.publish_rightLimb_controller
    rh_controller = WxContrApi.publish_rightHand_controller
    ll_controller = WxContrApi.publish_leftLimb_controller
    lh_controller = WxContrApi.publish_leftHand_controller
    # # 2.2 定义参数, 参数格式可查看函数获取
    # # 左臂参数
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
    l_names = ['' for _ in range(7)]
    r_names = ['' for _ in range(7)]

    lh_names = ['', '', '', '', '', '', '', '', '', '']
    rh_names = ['', '', '', '', '', '', '', '', '', '']

    l_command_1 = [0, 0, -1.65, -1.8, 2, 0, 0]  # 中间过程 1
    l_command_2 = [0.1, 0, -1.2, -1.85, 2.38, 0, 0]  # 中间过程 2
    l_command_3 = [0.3, 0, -1.2, -1.46, 2.38, 0, 0]  # 最终把手位置
    
    r_command_1 = [0, 0, 1.65, -1.8, -2, 0, 0]  # 中间过程 1
    r_command_2 = [-0.1, 0, 1.2, -1.85, -2.38, 0, 0]  # 中间过程 2
    r_command_3 = [-0.3, 0, 1.2, -1.46, -2.38, 0, 0]  # 最终把手位置
    
    wj = WxWebotsApi.WxWebotsJoints()
    wj.sub_leftHand_joint_states()
    wj.sub_leftLimb_joint_states()

    lh_command = [-0.0005440629177677493, 0.2, 
    0.4, 0.6, 
    0.4, 0.6, 
    0.4, 0.6, 
    0.4, 0.6]

    rh_command = [-0.0005440629177677493, 0.2, 
    0.4, 0.6, 
    0.4, 0.6, 
    0.4, 0.6, 
    0.4, 0.6]
    # 2.3 发布到相应的topic
    # ll_controller(l_names, l_command)
    cur_time = 0.01
    is_status_1 = False
    is_status_2 = False
    is_status_3 = False
    is_status_4 = False
    status = 1

    x1 = []
    x2 = []
    x3 = []
    x4 = []
    x5 = []
    x6 = []
    x7 = []
    tims = []

    while True:
        # ll_controller(l_names, l_command_4)
        if status == 1:
            ll_controller(l_names, l_command_1)
            rl_controller(r_names, r_command_1)
        elif status == 2:
            ll_controller(l_names, l_command_2)
            rl_controller(r_names, r_command_2)
        elif status == 3:
            ll_controller(l_names, l_command_3)
            rl_controller(r_names, r_command_3)
        elif status == 4:
            lh_controller(lh_names, lh_command)
            rh_controller(rh_names, rh_command)
        elif status == 5:
            ws.walk_forward_step(5) # 向前走

        leftHand_joint_states = wj.leftHand_joint_states
        leftLimb_joint_states = wj.leftLimb_joint_states
        if leftLimb_joint_states:
            cur_time += 0.01
            if cur_time >= 3000:  # 及时关闭步态
                ws.stop()
                break
            # 打印变化曲线
            tims.append(cur_time)
            position = leftLimb_joint_states.position
            x1.append(position[0])
            x2.append(position[1])
            x3.append(position[2])
            x4.append(position[3])
            x5.append(position[4])
            x6.append(position[5])
            x7.append(position[6])

        if leftLimb_joint_states:
            position = list(leftLimb_joint_states.position)
            if is_check_position_same(position, l_command_1) and not is_status_1:
                status = 2
                is_status_1 = True
            elif is_check_position_same(position, l_command_2) and not is_status_2:
                status = 3
                is_status_2 = True
            elif is_check_position_same(position, l_command_3) and not is_status_3:
                status = 4
                is_status_3 = True

        if leftHand_joint_states:
            position = list(leftHand_joint_states.position)
            if is_check_position_same(position, lh_command) and not is_status_4:
                status = 5
                is_status_4 = True
            
            # print(rightLimb_joint_states.name)
            # print(rightLimb_joint_states.velocity)
            # print(rightLimb_joint_states.effort)
    # 3 webots中查看变化
    plt.plot(tims, x1, ls="-", lw=2, label="x1")
    plt.plot(tims, x2, ls="-", lw=2, label="x2")
    plt.plot(tims, x3, ls="-", lw=2, label="x3")
    plt.plot(tims, x4, ls="-", lw=2, label="x4")
    plt.plot(tims, x5, ls="-", lw=2, label="x5")
    plt.plot(tims, x6, ls="-", lw=2, label="x6")
    plt.plot(tims, x7, ls="-", lw=2, label="x7")
    plt.legend()

    plt.show()
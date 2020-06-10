"""
    Task6: Push Cart
        推平板车 30分, 运动控制模式

        1、客厅3放置有1个平板车，平板车前方地面两侧有2道红色引导线。
        2、本任务要求，Walker双手握住平板车把手，直线推动平板车，直至平板车越过红色引导线。行走过程中，平板车必须始终处于两道引导线中间范围。
        3、调用PushCart任务，Walker出现于客厅3的固定位置，平板车与Walker的相对位置固定，Walker无需移动足部位置即可接触到平板车把手，可通过运动规划手臂完成任务。
        4、任务6的总运行时间不超过3分钟，逾时则任务挑战失败。

    Author: Dongbox
"""
import time

import rospy
from utils.walker_scenes import scene_task6_PushCart
from utils import WxWebotsApi
import numpy as np


def is_check_position_same(cur_position, check_position):
    cur_p = np.array(list(cur_position))
    check_p = np.array(list(check_position))
    res = abs(check_p - cur_p)
    if sum(res <= 0.004) >= 7:
        return True
    else:
        return False


def task6():
    """
        所用关节范围对应表
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
        LShoulderPitch : RShoulderPitch 反

        # LFinger -1.00E-06 1.5708 0.5
    """
    # 1.切换到对应场景
    # scene_task6_PushCart()
    # 2.1 获取所需的控制器
    WxContrApi = WxWebotsApi.WxWebotsControllers()
    ws = WxWebotsApi.WxWebotsStepFunc()
    # ws.standing()
    rl_controller = WxContrApi.publish_rightLimb_controller
    rh_controller = WxContrApi.publish_rightHand_controller
    ll_controller = WxContrApi.publish_leftLimb_controller
    lh_controller = WxContrApi.publish_leftHand_controller
    # # 2.2 定义参数, 参数格式可查看函数获取
    # # 左臂参数

    l_names = ['' for _ in range(7)]
    r_names = ['' for _ in range(7)]

    lh_names = ['', '', '', '', '', '', '', '', '', '']
    rh_names = ['', '', '', '', '', '', '', '', '', '']

    l_command_1 = [0, 0, -1.65, -1.8, 2, 0, 0]  # 中间过程 1
    l_command_2 = [0.1, 0, -1.2, -1.85, 2.38, 0, 0]  # 中间过程 2
    l_command_3 = [0.3, 0, -1.2, -1.5, 2.38, 0, 0]  # 最终把手位置

    l_commands = [l_command_1, l_command_2, l_command_3]
    r_command_1 = [0, 0, 1.65, -1.8, -2, 0, 0]  # 中间过程 1
    r_command_2 = [-0.1, 0, 1.2, -1.85, -2.38, 0, 0]  # 中间过程 2
    r_command_3 = [-0.3, 0, 1.2, -1.5, -2.38, 0, 0]  # 最终把手位置

    r_commands = [r_command_1, r_command_2, r_command_3]
    wj = WxWebotsApi.WxWebotsJoints()
    wj.sub_leftHand_joint_states()
    wj.sub_rightHand_joint_states()
    wj.sub_leftLimb_joint_states()
    wj.sub_rightLimb_joint_states()

    lh_command = [-0.0005440629177677493, 0.2,
                  0.7, 0.8,
                  0.7, 0.8,
                  0.7, 0.8,
                  0.7, 0.8]

    rh_command = [-0.0005440629177677493, 0.2,
                  0.7, 0.8,
                  0.7, 0.8,
                  0.7, 0.8,
                  0.7, 0.8]
    is_status_1 = False
    is_status_2 = False
    is_status_3 = False
    is_status_4 = False
    is_status_5 = False
    status = 1
    is_l_over = False
    is_r_over = False
    l_commands = WxWebotsApi.line_fit(l_commands, timeline=1000000, is_plot=True)
    r_commands = WxWebotsApi.line_fit(r_commands, timeline=1000000)
    while not rospy.is_shutdown():
        if not is_l_over and not is_r_over:
            for l_cmd, r_cmd in zip(l_commands, r_commands):
                ll_controller(l_names, l_cmd)
                rl_controller(r_names, r_cmd)
    # try:
    #
    #
    #         # 2.3 发布到相应的topic
    #         # if status == 1:
    #         #     ll_controller(l_names, l_command_1)
    #         #     rl_controller(r_names, r_command_1)
    #         # elif status == 2:
    #         #     ll_controller(l_names, l_command_2)
    #         #     rl_controller(r_names, r_command_2)
    #         # elif status == 3:
    #         #     ll_controller(l_names, l_command_3)
    #         #     rl_controller(r_names, r_command_3)
    #         # elif status == 4:
    #         #     lh_controller(lh_names, lh_command)
    #         #     rh_controller(rh_names, rh_command)
    #         # elif status == 5 and not is_status_5:
    #         #     is_status_5 = True
    #         #     status = 6
    #         #     ws.start()  # 向前走
    #         #     linear_x = 0.1
    #         #     linear_y = 0.005
    #         #     angular_z = 0.005
    #         #     ws.vel(linear_x, linear_y, angular_z)  # 设置前向速度
    #         # elif status == 6:
    #         #     if ws.cur_step_num - ws.last_step_num >= 15:
    #         #         ws.stop()
    #         #         break
    #         #
    #         # leftHand_joint_states = wj.leftHand_joint_states
    #         # rightHand_joint_states = wj.rightHand_joint_states
    #         # leftLimb_joint_states = wj.leftLimb_joint_states
    #         # rightLimb_joint_states = wj.rightLimb_joint_states
    #         #
    #         # if leftLimb_joint_states and rightLimb_joint_states:
    #         #     l_position = list(leftLimb_joint_states.position)
    #         #     r_position = list(rightLimb_joint_states.position)
    #         #     if is_check_position_same(l_position, l_command_1) and \
    #         #             is_check_position_same(r_position, r_command_1) and \
    #         #             not is_status_1:
    #         #         status = 2
    #         #         is_status_1 = True
    #         #     elif is_check_position_same(l_position, l_command_2) and \
    #         #             is_check_position_same(r_position, r_command_2) and \
    #         #             not is_status_2:
    #         #         status = 3
    #         #         is_status_2 = True
    #         #     elif is_check_position_same(l_position, l_command_3) and \
    #         #             is_check_position_same(r_position, r_command_3) and \
    #         #             not is_status_3:
    #         #         status = 4
    #         #         is_status_3 = True
    #         #
    #         # if leftHand_joint_states and rightHand_joint_states:
    #         #     l_position = list(leftHand_joint_states.position)
    #         #     r_position = list(rightHand_joint_states.position)
    #         #     if is_check_position_same(l_position, lh_command) and \
    #         #             is_check_position_same(r_position, rh_command) and \
    #         #             not is_status_4:
    #         #         status = 5
    #         #         is_status_4 = True
    #         #         time.sleep(3)
    #
    # except Exception:
    #     pass
    # finally:
    #     ws.stop()
    #     ws.standing()

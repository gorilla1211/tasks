"""
    切换到task2 -> 拿饮料罐  20分

    运动控制模式

    1、房间2放置有1张桌子，上面有5个体积大小相同但罐身装饰不同的饮料罐。
    2、本任务要求，Walker需从中拿出指定的饮料罐离开桌面并明显高于其他饮料罐保持3秒或以上。
    3、调用GraspCup任务，Walker位于房间2的固定位置，桌子与Walker的相对位置固定，Walker无需移动足部位置即可接触到饮料罐，可通过运动规划手臂完成任务。
    4、任务2的总运行时间不超1分钟，逾时则任务挑战失败。

    Author: DaLiang
"""
import rospy
from utils.walker_scenes import scene_task2_GraspCup
from utils import WxWebotsApi
import numpy as np
import time

WxContrApi = WxWebotsApi.WxWebotsControllers()
# 左手臂控制器
ll_controller = WxContrApi.publish_leftLimb_controller
# 右手臂控制器
rl_controller = WxContrApi.publish_rightLimb_controller
# 左手指控制器
lh_controller = WxContrApi.publish_leftHand_controller
# 右手指控制器
rh_controller = WxContrApi.publish_rightHand_controller
"""
    左手臂参数

    LShoulderPitch  -0.785  3.14        96.2    # 肩 举起/放下臂
    LShoulderRoll   -1.57   0.0175      96.2    # 肩 打开/关闭臂
    LShoulderYaw    -1.92   1.92        96.2    # 肩 旋转臂

    LElbowRoll      -2.27   1.00E-06    96.2    # 肘 打开/关闭肘
    LElbowYaw       -2.36   2.36        27.3    # 肘 旋转肘

    LWristPitch     -0.351  0.351       27.3    # 腕 上/下切
    LWristRoll      -0.351  0.351       27.3    # 腕 手心/背翻
"""
ll_names = ['LShoulderPitch', 'LShoulderRoll', 'LShoulderYaw',
            'LElbowRoll', 'LElbowYaw',
            'LWristPitch', 'LWristRoll']

"""
    右臂参数
    
    RShoulderPitch  -1.00E-06   1.5708      96.2    # 肩 举起/放下臂
    RShoulderRoll   -3.14       0.785       96.2    # 肩 打开/关闭臂
    RShoulderYaw    -1.57       0.0175      96.2    # 肩 旋转臂

    RElbowRoll      -1.92       1.92        96.2    # 肘 打开/关闭肘
    RElbowYaw       -2.27       1.00E-06    27.3    # 肘 旋转肘

    RWristPitch     -0.351      0.351       27.3    # 腕 上/下切
    RWristRoll      -2.36       2.36        27.3    # 腕 手心/背翻
"""
rl_names = ['RShoulderPitch', 'RShoulderRoll', 'RShoulderYaw',
            'RElbowRoll', 'RElbowYaw',
            'RWristPitch', 'RWristRoll']

"""
    左手指参数
    LFinger         -1.00E-06   1.5708      0.5

    LFirstFinger1
    LFirstFinger2

    LSecondFinger1
    LSecondFinger2

    LThirdFinger1
    LThirdFinger2

    LForthFinger1
    LForthFinger2

    LFifthFinger1
    LFifthFinger2
"""
lh_names = ['LFirstFinger1', 'LFirstFinger2', 'LSecondFinger1', 'LSecondFinger2'
            'LThirdFinger1', 'LThirdFinger2', 'LForthFinger1', 'LForthFinger2',
            'LFifthFinger1', 'LFifthFinger2']

"""
    右手指参数
    RFinger         -0.351      0.351      0.5

    RFirstFinger1
    RFirstFinger2

    RSecondFinger1
    RSecondFinger2

    RThirdFinger1
    RThirdFinger2

    RForthFinger1
    RForthFinger2

    RFifthFinger1
    RFifthFinger2
"""
rh_names = ['RFirstFinger1', 'RFirstFinger2', 'RSecondFinger1', 'RSecondFinger2'
            'RThirdFinger1', 'RThirdFinger2', 'RForthFinger1', 'RForthFinger2',
            'RFifthFinger1', 'RFifthFinger2']


def capture_cup_1():
    ll_command_1 = [0, 0, 0, 0, 0, 0, 0]
    ll_command_2 = [0, -1.0, 0, 0, 0, 0, 0]
    ll_command_3 = [0, -1.0, -1.5, 0, 0, 0, 0]
    ll_command_4 = [0, -1.0, -1.5, -1.2, 0, 0, 0]
    ll_command_5 = [0, -0.5, -1.5, -1.2, 0.85, 0, 0]
    ll_command_6 = [0.2, -0.5, -1.5, -1.1, 0.85, 0, 0]

    ll_command_7 = [0.5, -0.5, -1.5, -1.1, 0.85, 0, 0]
    commands = [ll_command_1, ll_command_2, ll_command_3, ll_command_4, ll_command_5, ll_command_6]
    ll_commands = WxWebotsApi.line_fit(commands, timeline=1000000)

    lh_command1 = [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7]
    lh_command2 = [0.5, 0.1, 0.5, 0.2, 0.5, 0.2, 0.5, 0.2, 0.5, 0.2]

    ll_is_over = False
    lh_is_over = False
    while not rospy.is_shutdown():
        if not ll_is_over:
            for cmd in ll_commands:
                ll_controller(ll_names, cmd, mode=5)
            ll_is_over = True
        elif not lh_is_over:
            for _ in range(500000):
                lh_controller(lh_names, lh_command1, mode=5)
            lh_is_over = True
        else:
            print('1号杯已握紧')
            lh_controller(lh_names, lh_command2, mode=7)
            for cmd in WxWebotsApi.line_fit([ll_command_7], timeline=500000):
                ll_controller(ll_names, cmd, mode=5)
            print('1号杯已拿起')
            break


def capture_cup_2():
    ll_command_1 = [0, 0, 0, 0, 0, 0, 0]
    ll_command_2 = [0, -1.0, 0, 0, 0, 0, 0]
    ll_command_3 = [0, -1.0, -1.5, 0, 0, 0, 0]
    ll_command_4 = [0, -1.0, -1.5, -1.5, 0, 0, 0]
    ll_command_5 = [0.5, -1.0, -1.5, -1.5, 0, 0, 0]
    ll_command_6 = [0.5, -0.5, -1.5, -1.5, 0, 0, 0]
    ll_command_7 = [0.5, 0, -1.5, -1.5, 0, 0, 0]
    ll_command_8 = [0.5, 0, -1.5, -1.5, 0.9, 0, 0]
    ll_command_9 = [0.2, 0, -1.5, -1.5, 0.9, 0, 0]
    ll_command_10 = [0.1, 0, -1.4, -1.5, 1.2, 0, 0]
    ll_command_11 = [0.1, 0, -1.4, -1.5, 1.4, 0, 0]
    ll_command_12 = [0.2, 0, -1.4, -1.4, 1.4, 0, 0.05]

    ll_command_13 = [0.5, 0, -1.4, -1.4, 1.4, 0, 0.05]
    ll_command_s = [ll_command_1, ll_command_2, ll_command_3, ll_command_4, ll_command_5, ll_command_6, ll_command_7,
                    ll_command_8, ll_command_9, ll_command_10, ll_command_11, ll_command_12]
    ll_commands = WxWebotsApi.line_fit(ll_command_s, timeline=2000000)

    lh_command1 = [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7]
    lh_command2 = [0.5, 0.1, 0.5, 0.2, 0.5, 0.2, 0.5, 0.2, 0.5, 0.2]

    ll_is_over = False
    lh_is_over = False
    while not rospy.is_shutdown():
        if not ll_is_over:
            for cmd in ll_commands:
                ll_controller(ll_names, cmd, mode=5)
            ll_is_over = True
        elif not lh_is_over:
            for _ in range(500000):
                lh_controller(lh_names, lh_command1, mode=5)
            lh_is_over = True
        else:
            print('2号杯已握紧')
            lh_controller(lh_names, lh_command2, mode=7)
            for cmd in WxWebotsApi.line_fit([ll_command_13], timeline=500000):
                ll_controller(ll_names, cmd, mode=5)
            print('2号杯已拿起')
            break


def capture_cup_4():
    rl_command_1 = [0, 0, 0, 0, 0, 0, 0]
    rl_command_2 = [0, -1.0, 0, 0, 0, 0, 0]
    rl_command_3 = [0, -1.0, 1.5, 0, 0, 0, 0]
    rl_command_4 = [0, -1.0, 1.5, -1.5, 0, 0, 0]
    rl_command_5 = [-0.3, -1.0, 1.5, -1.5, 0, 0, 0]

    rl_command_6 = [-0.3, -1.0, 1.5, -1.5, 0, 0, 0]
    rl_command_7 = [-0.2, -0.5, 1.5, -1.5, -0.65, 0, 0.1]
    rl_command_8 = [-0.2, -0.4, 1.3, -1.3, -0.95, 0, 0.1]

    rl_command_9 = [-0.5, -0.4, 1.3, -1.3, -0.95, 0, 0.1]
    rl_command_s_1 = [rl_command_1, rl_command_2, rl_command_3, rl_command_4, rl_command_5]
    rl_commands_1 = WxWebotsApi.line_fit(rl_command_s_1, timeline=1000000)

    rl_command_s_2 = [rl_command_5, rl_command_6, rl_command_7, rl_command_8]
    rl_commands_2 = WxWebotsApi.line_fit(rl_command_s_2, timeline=1000000)

    rh_command1 = [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7]
    rh_command2 = [0.5, 0.1, 0.5, 0.2, 0.5, 0.2, 0.5, 0.2, 0.5, 0.2]

    rl_is_over_1 = False
    rl_is_over_2 = False
    rh_is_over = False
    while not rospy.is_shutdown():
        if not rl_is_over_1:
            for cmd in rl_commands_1:
                rl_controller(rl_names, cmd, mode=5)
            rl_is_over_1 = True
        elif not rl_is_over_2:
            for cmd in rl_commands_2:
                rl_controller(rl_names, cmd, mode=5)
            rl_is_over_2 = True
        elif not rh_is_over:
            for _ in range(500000):
                rh_controller(rh_names, rh_command1, mode=5)
            rh_is_over = True
        else:
            print('4号杯已握紧')
            rh_controller(rh_names, rh_command2, mode=7)
            for cmd in WxWebotsApi.line_fit([rl_command_9], timeline=500000):
                rl_controller(rl_names, cmd, mode=5)
            print('4号杯已拿起')
            break


def capture_cup(cup_num):
    if cup_num == 1:
        print('拿1号杯')
        capture_cup_1()
    elif cup_num == 2:
        print('拿2号杯')
        capture_cup_2()
    elif cup_num == 4:
        print('拿4号杯')
        capture_cup_4()
    else:
        print(str(cup_num) + ': The cup can does not exist!')


def task2(cup_num):
    scene_task2_GraspCup()
    time.sleep(4)
    capture_cup(cup_num=cup_num)

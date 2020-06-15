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
lh_controller = WxContrApi.publish_leftHand_controller

"""
    左手臂参数
    X 轴: 红色      Y 轴: 绿色      Z 轴: 蓝色  

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

lh_command = [0, 0,
              0, 0,
              0, 0,
              0, 0,
              0, 0]


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

    lh_command0 = [0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0]
    lh_command1 = [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7]

    lh_command2 = [0.2, 0.2, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0, 0]

    lh_command_s = [lh_command0]
    lh_commands = WxWebotsApi.line_fit(commands=lh_command_s, timeline=100000)

    ll_is_over = False
    lh_is_over = False
    while not rospy.is_shutdown():
        if not ll_is_over:
            for cmd in ll_commands:
                ll_controller(ll_names, cmd)
            ll_is_over = True
        elif not lh_is_over:
            for cmd in lh_commands:
                lh_controller(lh_names, cmd)
            lh_is_over = True
        else:
            # lh_controller(lh_names, lh_command2, mode=7)
            # ll_controller(ll_names, ll_command_7, mode=7)
            break


def capture_cup(cup_num):
    if cup_num == 1:
        print('Take the first cup')
        capture_cup_1()
    else:
        print(str(cup_num) + ': The cup can does not exist!')


def task2(cup_num):
    scene_task2_GraspCup()
    time.sleep(4)
    capture_cup(cup_num=cup_num)
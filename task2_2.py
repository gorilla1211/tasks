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
from utils.walker_scenes import scene_task2_GraspCup
from utils.walker_tops_srvs import BuildConnect, WalkerWebotsSub
from utils import WxWebotsApi
from utils.walker_types import JointCommand
import math


def task2_2():
    # 1.切换到特定的场景
    scene_task2_GraspCup()
    # 2.1获取相应的控制器
    WxCotroApi = WxWebotsApi.WxWebotsControllers()
    WxJointApi = WxWebotsApi.WxWebotsJoints()
    ll_controller = WxCotroApi.publish_leftLimb_controller
    
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
    time = 0.01
    # 左臂参数
    l_names = ['' for _ in range(7)]
    l_command = [0 for _ in range(7)]


    # 2.3 发布到相应的topic
    while not rospy.is_shutdown():
        # l_command[2] = 1.85  # LShoulderPitch
        # l_command[1] = -0.7     # 肩 打开/关闭臂
        # l_command[3] = 1.12     # 肘 打开/关闭肘
        # l_command[0] = 1.1      # 肩 举起/放下臂
        # l_command[4] = -0.5     # 肘 旋转肘
        for i in range(1,300000):
            l_command[1] = -0.9
            l_command[2] = -1.3
            ll_controller(l_names, l_command)        
        for i in range(1,300000):
            l_command[4] =  -0.6
            ll_controller(l_names, l_command)
        for i in range(1,300000):
            l_command[3] = -1.0
            ll_controller(l_names, l_command)
        for i in range(1,300000):
            l_command[3] = -1.3
            ll_controller(l_names, l_command)
        for i in range(1,100000):
            l_command[3] = -1.4
            ll_controller(l_names, l_command)
        l_command[0] = -0.3
        # l_command = [0.9,-0.2,-1.8,-1.2,2.3,0,-0.35]
        ll_controller(l_names, l_command)  # 传入参数控制左臂
        
        # LeftLimb_joint_states = WxJointApi.leftLimb_joint_states  #订阅左臂信息
        # if LeftLimb_joint_states:
        #     print(LeftLimb_joint_states)
            
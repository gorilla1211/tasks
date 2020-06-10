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


def task1():
    # 1.切换到特定的场景
    scene_task1_SwitchLight()
    # 2.1获取相应的控制器
    WxCotroApi = WxWebotsApi.WxWebotsControllers()
    WxJointApi = WxWebotsApi.WxWebotsJoints()
    ll_controller = WxCotroApi.publish_leftLimb_controller
    ll_sub_states = WxJointApi.sub_leftLimb_joint_states
    # 2.2 定义参数,参数格式可查看函数获取
    # time
    time = 0.01
    # 左臂参数
    l_names = ['' for _ in range(7)]
    l_command = [0 for _ in range(7)]
    l_mode = 6

    ll_sub_states()
    # 2.3 发布到相应的topic
    while not rospy.is_shutdown():
        l_command[2] = 1.85  # LShoulderPitch
        l_command[1] = -0.7     # 肩 打开/关闭臂
        l_command[3] = 1.12     # 肘 打开/关闭肘
        l_command[0] = 1.1      # 肩 举起/放下臂
        l_command[4] = -0.5     # 肘 旋转肘
        ll_controller(l_names, l_command)  # 传入参数控制左臂
        
        # LeftLimb_joint_states = WxJointApi.leftLimb_joint_states  #订阅左臂信息
        # if LeftLimb_joint_states:
        #     print(LeftLimb_joint_states)
            
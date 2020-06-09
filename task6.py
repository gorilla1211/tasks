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


def task6():
    # 1.切换到对应场景
    scene_task6_PushCart()
    # time
    # 2.1 获取所需的控制器
    WxCotroApi = WxWebotsApi.WxWebotsControllers()
    ll_controller = WxCotroApi.publish_leftLimb_controller
    rl_controller =WxCotroApi.publish_rightLimb_controller
    # # 2.2 定义参数, 参数格式可查看函数获取
    # # 左臂参数
    time = 0.01
    l_names = ['' for _ in range(7)]
    l_command = [0 for _ in range(7)]
    # # 右臂参数
    r_names = ['' for _ in range(7)]
    r_command = [0 for _ in range(7)]
    # 2.3 发布到相应的topic
    while not rospy.is_shutdown():
        time += 0.001
        l_command[0] = 0.5 * math.sin(time * 2 * math.pi / 1.0)  # LShoulderPitch
        for ll in range(6):
            l_command[ll + 1] = 0

        r_command[0] = 0.5 * math.sin(-time * 2 * math.pi / 1.0)  # LShoulderPitch
        for ll in range(6):
            r_command[ll + 1] = 0
           
        ll_controller(l_names, l_command)  # 传入参数控制左臂
        rl_controller(r_names, r_command)  # 传入参数控制右臂
    # 3 webots中查看变化
    
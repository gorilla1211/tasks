"""
    切换到task2 -> 拿饮料罐  20分

    运动控制模式
    
    1、房间2放置有1张桌子，上面有5个体积大小相同但罐身装饰不同的饮料罐。
    2、本任务要求，Walker需从中拿出指定的饮料罐离开桌面并明显高于其他饮料罐保持3秒或以上。
    3、调用GraspCup任务，Walker位于房间2的固定位置，桌子与Walker的相对位置固定，Walker无需移动足部位置即可接触到饮料罐，可通过运动规划手臂完成任务。
    4、任务2的总运行时间不超1分钟，逾时则任务挑战失败。

    Author: Daliang
"""
import rospy
from utils.walker_scenes import scene_task2_GraspCup
from utils.walker_tops_srvs import BuildConnect, WalkerWebotsSub
from utils import WxWebotsApi
from utils.walker_types import JointCommand
import numpy as np

count = 0

def is_check_position_same(cur_position, check_position):
    cur_p = np.array(list(cur_position))
    check_p = np.array(list(check_position))
    res = abs(check_p - cur_p)
    if sum(res<=0.004) >= 7:
        global count
        count += 1
        if count == 100:
            count = 0
            return True
    return False

def task2(cup_num=1):
    # 切换到任务2场景
    scene_task2_GraspCup()

    WxCotroApi = WxWebotsApi.WxWebotsControllers()
    # 获得左右手控制器
    ll_controller = WxCotroApi.publish_leftLimb_controller
    rl_controller = WxCotroApi.publish_rightLimb_controller

    # 获得左右手指的控制器
    lh_controller = WxCotroApi.publish_leftHand_controller
    rh_controller = WxCotroApi.publish_rightHand_controller


    """
        X 轴: 红色      Y 轴: 绿色      Z 轴: 蓝色  

        LShoulderPitch  -0.785  3.14        96.2    # 肩 举起/放下臂
        LShoulderRoll   -1.57   0.0175      96.2    # 肩 打开/关闭臂
        LShoulderYaw    -1.92   1.92        96.2    # 肩 旋转臂

        LElbowRoll      -2.27   1.00E-06    96.2    # 肘 打开/关闭肘
        LElbowYaw       -2.36   2.36        27.3    # 肘 旋转肘

        LWristPitch     -0.351  0.351       27.3    # 腕 上/下切
        LWristRoll      -0.351  0.351       27.3    # 腕 手心/背翻
    """

    # 定义参数, 参数格式可查看函数获取
    # 左臂参数
    l_names = ['LShoulderPitch', 'LShoulderRoll', 'LShoulderYaw',
               'LElbowRoll', 'LElbowYaw',
               'LWristPitch', 'LWristRoll']
    l_command_0 = [-2.7476229711559662e-05, 0.000548553138199625, 4.0090237114409884e-06,
                   -0.0002707919077573215, 1.6172207976904167e-06,
                   -1.6360476324464147e-05, 0.00010110325467476545]
    l_command_1 = [-0.7, -1.0, -1.3, 0, 0, -1.6360476324464147e-05, 0.00010110325467476545]
    l_command_2 = [-0.7, -1.0, -1.3, -1.5, 0, -1.6360476324464147e-05, 0.00010110325467476545]        
    l_command_3 = [0.5, -0.8, -1.3, -1.0, 0.5, -1.6360476324464147e-05, 0.00010110325467476545]
    l_command_4 = [0.4, -0.5, -1.3, -1.0, 0.5, -1.6360476324464147e-05, 0.00010110325467476545]
    l_command_5 = [0.48, -0.38, -1.3, -0.9, 0.9, -1.6360476324464147e-05, 0.00010110325467476545]
    l_command_6 = [0.50, -0.35, -1.3, -0.85, 0.9, -1.6360476324464147e-05, 0.00010110325467476545]
    l_commands = [l_command_1, l_command_2, l_command_3, l_command_4,
                  l_command_5, l_command_6]

    # 定义标签
    is_l_over = False
    is_r_over = False
    ll_commands = WxWebotsApi.line_fit(l_commands, timeline=1000000, is_plot=False)
    while not rospy.is_shutdown():
        if not is_l_over and not is_r_over:
            # 发布到topic
            for ll_cmd in ll_commands:
                ll_controller(l_names, ll_cmd)
            is_l_over = True
            is_r_over = True

    # 左手指参数
    """
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
                'LThirdFinger1', 'LThirdFinger2', 'LForthFinger1', 'LForthFinger2', 'LFifthFinger1', 'LFifthFinger2']

    lh_command_1 = [0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0]

    lh_mode = 5

    wj = WxWebotsApi.WxWebotsJoints()
    wj.sub_leftHand_joint_states()
    wj.sub_leftLimb_joint_states()

    is_status_1 = True
    is_status_2 = False
    is_status_3 = False
    is_status_4 = False
    is_status_5 = False
    is_status_6 = False
    is_status_7 = False
    is_status_8 = False
    status = 1
    
    if cup_num == 1:
        while True:
            if status == 1:
                ll_controller(names=l_names, command=l_command_1)
            elif status == 2:
                ll_controller(names=l_names, command=l_command_2)
            elif status == 3:
                ll_controller(names=l_names, command=l_command_3)
            elif status == 4:
                ll_controller(names=l_names, command=l_command_4)
            elif status == 5:
                ll_controller(names=l_names, command=l_command_5)
            elif status == 6:
                ll_controller(names=l_names, command=l_command_6)
            elif status == 7:
                break

            leftLimb_joint_states = wj.leftLimb_joint_states
            leftHand_joint_states = wj.leftHand_joint_states

            if leftLimb_joint_states:
                position = list(leftLimb_joint_states.position)
                if is_check_position_same(position, l_command_1) and is_status_1:
                    status = 2
                    is_status_1 = False
                    is_status_2 = True
                    print('完成第1步动作')
                elif is_check_position_same(position, l_command_2) and is_status_2:
                    status = 3
                    is_status_2 = False
                    is_status_3 = True
                    print('完成第2步动作')
                elif is_check_position_same(position, l_command_3) and is_status_3:
                    status = 4
                    is_status_3 = False
                    is_status_4 = True
                    print('完成第3步动作')
                elif is_check_position_same(position, l_command_4) and is_status_4:
                    status = 5
                    is_status_4 = False
                    is_status_5 = True
                    print('完成第4步动作')
                elif is_check_position_same(position, l_command_5) and is_status_5:
                    status = 6
                    is_status_5 = False
                    is_status_6 = True
                    print('完成第5步动作')
                elif is_check_position_same(position, l_command_6) and is_status_6:
                    status = 7
                    is_status_6 = False
                    is_status_7 = True
                    print('完成第6步动作')
            
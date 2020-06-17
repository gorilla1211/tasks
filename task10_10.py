import rospy
from utils.walker_scenes import scene_task10_OpenFridge
from utils.walker_func import WxWebotsApi
import time



def task10_10():
    time.sleep(5)
    scene_task10_OpenFridge()
    time.sleep(5)
    ws = WxWebotsApi.WxWebotsStepFunc()
    WxContrApi = WxWebotsApi.WxWebotsControllers()
    rl_controller = WxContrApi.publish_rightLimb_controller
    rh_controller = WxContrApi.publish_rightHand_controller
    lj_controller = WxContrApi.publish_rightHand_controller
    #forward_finished = ws.walk_forward_step(1,-0.25)
    rl_names = ['RShoulderPitch', 'RShoulderRoll', 'RShoulderYaw',
            'RElbowRoll', 'RElbowYaw',
            'RWristPitch', 'RWristRoll']
    rl_command = [0 for _ in range(7)]

    rh_names = ['LFirstFinger1', 'LFirstFinger2', 'LSecondFinger1', 'LSecondFinger2'
            'LThirdFinger1', 'LThirdFinger2', 'LForthFinger1', 'LForthFinger2',
            'LFifthFinger1', 'LFifthFinger2']
    rh_command = ['' for _ in range(10)]

    """
        所用关节范围对应表

        RShoulderPitch -1.00E-06 1.5708   96.2
        RShoulderRoll  -3.14     0.785    96.2
        RShoulderYaw   -1.57     0.0175   96.2
        RElbowRoll     -1.92     1.92     96.2
        RElbowYaw      -2.27     1.00E-06 27.3
        RWristRoll    -2.36     2.36     27.3
        RWristPitch    -0.351    0.351    27.3

        LShoulderYaw : RShoulderYaw 反
        LElbowYaw : RElbowYaw 反
        LShoulderPitch : RShoulderPitch 反

        # LFinger -1.00E-06 1.5708 0.5
    """
    rl_command1 = [0, 0, 0, 0, 0, 0,0]
    rl_command2 = [-0.8, -0.2, 0, 0, 0, 0, 0]
    rl_command3 = [-1.2, -0.4, 0.1, 0, 0, 0, 0]
    rl_command4 = [-1.57, 0.1, 0.1, 0, 0.2, 0,0]
    rl_command5 = [-1.57, 0.0, 0.6, 0, 0.7, 0.4, 0]
    # rl_command0 = [0,0,0,0,]
    lj_commadn1 = []
    rh_command2 = [0.1, 0,
                   0.5, 0.5,
                   0.5, 0.5,
                   0.5, 0.5,
                   0.5, 0.5]
    rl_command0 = [5,5,5,5,5,5,5]
    rl_commands = [rl_command1, rl_command2 ,rl_command3, rl_command4]
    rl_commands2 = [rl_command5]
    rl_command_bezier = WxWebotsApi.line_fit(rl_commands,timeline=1500000)
    rl_command_bezier2 = WxWebotsApi.line_fit(rl_commands2,timeline=300000)
    rl_is_over = False
    rl2_is_over = False
    rh1_is_over = False
    rh2_is_over = False
    rh3_is_over = False
    rh4_is_over = False
    if ws.walk_forward_step(step_num=1,linear_x=-0.30):
        while not rospy.is_shutdown():
            # l_command[2] = 1.85  # LShoulderPitch
            # l_command[1] = -0.7     # 肩 打开/关闭臂
            # l_command[3] = 1.12     # 肘 打开/关闭肘
            # l_command[0] = 1.1      # 肩 举起/放下臂
            # l_command[4] = -0.5     # 肘 旋转肘
            time.sleep(12)
            if not rh1_is_over:
                for i in range(1,50000):
                    rh_command = [0.0, 0.0,
                    0.4, 0.4,
                    0.4, 0.4,
                    0.4, 0.4,
                    0.4, 0.4]
                    rh_controller(rh_names, rh_command,mode=5)
                rh1_is_over = True
            elif not rl_is_over:
                for cmd in rl_command_bezier:
                    rl_controller(rl_names, cmd, mode=5)
                rl_is_over = True
            elif not rh2_is_over:
                for i in range(1,50000):
                    rh_command = [0.3, 0.3,
                    0.8, 0.9,
                    0.8, 0.9,
                    0.8, 0.9,
                    0.8, 0.9]
                    rh_controller(rh_names, rh_command,mode=5)
                rh2_is_over = True
            elif not rh3_is_over:
                for i in range(1,50000):
                    rh_command = [0.8, 0.8,
                    1.57, 1.57,
                    1.57, 1.57,
                    1.57, 1.57,
                    1.57, 1.57]
                    rh_controller(rh_names, rh_command,mode=5)
                rh3_is_over = True 
            elif not rl2_is_over:
                rh_controller(rh_names, rh_command2, mode=7)
                rl2_is_over = True
            # elif not rh4_is_over:
            #     # if ws.walk_forward_step(step_num=3,linear_x=-0.2):
            #     #     rh4_is_over = True
            else:
                break
            # for i in range(1,300000):
            #     r_command[0] = -1.6
            #     rl_controller(r_names, r_command)
            # for i in range(1,300000):
            #     r_command[1] = 0.1
            # for i in range(1,300000):
            #     r_command[3] = -1
            #     rl_controller(r_names, r_command)
            # l_command = [0.9,-0.2,-1.8,-1.2,2.3,0,-0.35]
            # 传入参数控制左臂
        # while True:
        #     if 
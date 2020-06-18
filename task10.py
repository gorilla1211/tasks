import rospy
from utils.walker_scenes import scene_task10_OpenFridge
from utils.walker_func import WxWebotsApi
import time


def task10():
    time.sleep(5)
    scene_task10_OpenFridge()
    time.sleep(5)
    ws = WxWebotsApi.WxWebotsStepFunc()
    WxContrApi = WxWebotsApi.WxWebotsControllers()
    rl_controller = WxContrApi.publish_rightLimb_controller
    rh_controller = WxContrApi.publish_rightHand_controller
    ll_controller = WxContrApi.publish_leftLimb_controller
    rl_names = ['RShoulderPitch', 'RShoulderRoll', 'RShoulderYaw',
                'RElbowRoll', 'RElbowYaw',
                'RWristPitch', 'RWristRoll']

    rh_names = ['LFirstFinger1', 'LFirstFinger2', 'LSecondFinger1', 'LSecondFinger2'
                                                                    'LThirdFinger1', 'LThirdFinger2', 'LForthFinger1',
                'LForthFinger2',
                'LFifthFinger1', 'LFifthFinger2']

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
    rl_command1 = [0, 0, 0, 0, 0, 0, 0]
    rl_command2 = [-0.8, -0.2, 0, 0, 0, 0, 0]
    rl_command3 = [-1.2, -0.4, 0.1, 0, 0, 0, 0]
    rl_command4 = [-1.57, 0.1, 0.1, 0, 0.2, 0, 0]

    ll_command1 = [0, 0, 0, 0, 0, 0, 0]
    ll_command2 = [1.5, 0.3, 0, 0, 0, 0, 0]
    ll_command3 = [1.5, 0.3, 0, -0.7, 0, 0, 0]
    ll_command_force = [0, 0, 0, -5, 0, 0, 0]

    rl_command0 = [0, 10, 0, -10, 0, 0, 5]
    rl_command0_0 = [-0.8891579791007349, -1.0053011435310661, 0.6744789912475938, -1.4038224546750395,
                     -0.058873321286748104, -0.06892831649295111, 0.8230945483382753]
    rl_command0_1 = [-0.8891579791007349, -1.0053011435310661, 0.6744789912475938, -1.1038224546750395,
                     -0.058873321286748104, -0.06892831649295111, 0.8230945483382753]
    rl_command0_2 = [-0.8891579791007349, -1.0053011435310661, 0.6744789912475938, -0.9038224546750395,
                     -0.058873321286748104, -0.06892831649295111, 0.8230945483382753]
    rh_command0 = [0.4, 0.4,
                   0.4, 0.4,
                   0.4, 0.4,
                   0.4, 0.4,
                   0.4, 0.4]

    rh_command1 = [1.0, 1.0,
                   1.2, 1.2,
                   1.2, 1.2,
                   1.2, 1.2,
                   1.2, 1.2]

    rh_command2 = [0.6, 0.6,
                   0.8, 0.8,
                   0.8, 0.8,
                   0.8, 0.8,
                   0.8, 0.8]

    rl_commands = [rl_command1, rl_command2, rl_command3, rl_command4]
    rl_commands2 = [rl_command0_1, rl_command0_2, rl_command1]
    ll_commands = [ll_command1, ll_command2, ll_command3]
    ll_command_bezier = WxWebotsApi.line_fit(ll_commands, timeline=500000)
    rl_command_bezier2 = WxWebotsApi.line_fit(rl_commands2, timeline=100000)
    rl_command_bezier = WxWebotsApi.line_fit(rl_commands, timeline=800000)
    rl_is_over = False
    rl2_is_over = False
    rh1_is_over = False
    rh2_is_over = False
    rh3_is_over = False
    rh4_is_over = False
    rh5_is_over = False
    rh6_is_over = False
    ll_is_over = False
    ll_names = ['LShoulderPitch', 'LShoulderRoll', 'LShoulderYaw',
                'LElbowRoll', 'LElbowYaw',
                'LWristPitch', 'LWristRoll']
    if ws.walk_forward_step(step_num=1, linear_x=-0.30):
        while not rospy.is_shutdown():
            # l_command[2] = 1.85  # LShoulderPitch
            # l_command[1] = -0.7     # 肩 打开/关闭臂
            # l_command[3] = 1.12     # 肘 打开/关闭肘
            # l_command[0] = 1.1      # 肩 举起/放下臂
            # l_command[4] = -0.5     # 肘 旋转肘
            time.sleep(12)
            if not rh1_is_over:
                for i in range(1, 50000):
                    rh_command = [0.01, 0.01,
                                  0.4, 0.4,
                                  0.4, 0.4,
                                  0.4, 0.4,
                                  0.4, 0.4]
                    rh_controller(rh_names, rh_command, mode=5)
                rh1_is_over = True
            elif not rl_is_over:
                for cmd in rl_command_bezier:
                    rl_controller(rl_names, cmd, mode=5)
                rl_is_over = True
            elif not rh2_is_over:
                for i in range(1, 50000):
                    rh_command = [0.3, 0.3,
                                  0.8, 0.9,
                                  0.8, 0.9,
                                  0.8, 0.9,
                                  0.8, 0.9]
                    rh_controller(rh_names, rh_command, mode=5)
                rh2_is_over = True
            elif not rh3_is_over:
                for i in range(1, 50000):
                    rh_command = [0.8, 0.8,
                                  1.57, 1.57,
                                  1.57, 1.57,
                                  1.57, 1.57,
                                  1.57, 1.57]
                    rh_controller(rh_names, rh_command, mode=5)
                rh3_is_over = True
            elif not rh4_is_over:
                # for cmd in rl_command_bezier2:
                #     rl_controller(rl_names, cmd, mode=5)
                first = rospy.get_time()
                last = rospy.get_time()
                rl_controller(rl_names, rl_command0, mode=7)
                while (last - first) < 0.45:
                    last = rospy.get_time()
                rl_controller(rl_names, rl_command0_0, mode=5)
                rh4_is_over = True
            elif not ll_is_over:
                for cmd in ll_command_bezier:
                    ll_controller(ll_names, cmd, mode=5)
                ll_is_over = True
            elif not rh6_is_over:
                for i in range(1, 60000):
                    rh_controller(rh_names, rh_command1, mode=5)
                for i in range(1, 60000):
                    rh_controller(rh_names, rh_command2, mode=5)
                rh6_is_over = True
            elif not rh5_is_over:
                for cmd in rl_command_bezier2:
                    rl_controller(rl_names, cmd, mode=5)
                rh5_is_over = True
            else:
                for i in range(1, 10000):
                    ll_controller(ll_names, ll_command1, mode=5)
                break
    '''步态旋转'''


    ws.start()
    first = rospy.get_time()
    last = rospy.get_time()
    ws.vel(angular_z=0.3)
    while (last - first)<2.0:
        ws.vel(angular_z=-0.2)
        last = rospy.get_time()
    ws.stop()


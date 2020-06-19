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
    ll_names = ['LShoulderPitch', 'LShoulderRoll', 'LShoulderYaw',
                'LElbowRoll', 'LElbowYaw',
                'LWristPitch', 'LWristRoll']

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

    '''
    rl_command1-4：抬起右手抓住冰箱
    '''
    rl_command1 = [0, 0, 0, 0, 0, 0, 0]
    rl_command2 = [-0.8, -0.2, 0, 0, 0, 0, 0]
    rl_command3 = [-1.2, -0.4, 0.1, 0, 0, 0, 0]
    rl_command4 = [-1.57, 0.1, 0.1, 0, 0.2, 0, 0]
    '''
    ll_command1-3：抬起左手扶住冰箱，并给冰箱向右边的惯性
    '''
    ll_command1 = [0, 0, 0, 0, 0, 0, 0]
    ll_command2 = [1.5, 0.3, 0, 0, 0, 0, 0]
    ll_command3 = [1.5, 0.3, 0, -0.67, 0, 0, 0]
    '''
    rl_command_force：右手给予冰箱的拉力
    '''
    rl_command_force = [0, 10, 0, -10, 0, 0, 5]
    '''
    rl_command5-7：右手拉到特定为之后取消力矩
    '''
    rl_command5 = [-0.8891579791007349, -1.0053011435310661, 0.6744789912475938, -1.4038224546750395,
                   -0.058873321286748104, -0.06892831649295111, 0.8230945483382753]
    rl_command6 = [-0.8891579791007349, -1.0053011435310661, 0.6744789912475938, -1.1038224546750395,
                   -0.058873321286748104, -0.06892831649295111, 0.8230945483382753]
    rl_command7 = [-0.8891579791007349, -1.0053011435310661, 0.6744789912475938, -0.9038224546750395,
                   -0.058873321286748104, -0.06892831649295111, 0.8230945483382753]
    '''手指初始位置'''
    rh_command1 = [0.01, 0.01,
                   0.4, 0.4,
                   0.4, 0.4,
                   0.4, 0.4,
                   0.4, 0.4]
    '''手指抓住冰箱'''
    rh_command2 = [0.8, 0.8,
                   1.57, 1.57,
                   1.57, 1.57,
                   1.57, 1.57,
                   1.57, 1.57]
    '''手指松手'''
    rh_command3 = [0.6, 0.6,
                   0.8, 0.8,
                   0.8, 0.8,
                   0.8, 0.8,
                   0.8, 0.8]

    rl_commands = [rl_command1, rl_command2, rl_command3, rl_command4]
    rl_commands2 = [rl_command6, rl_command7, rl_command1]
    ll_commands = [ll_command1, ll_command2, ll_command3]
    ll_commands2 = [ll_command3, ll_command2, ll_command1]
    ll_command_bezier = WxWebotsApi.line_fit(ll_commands, timeline=300000)
    rl_command_bezier2 = WxWebotsApi.line_fit(rl_commands2, timeline=80000)
    rl_command_bezier = WxWebotsApi.line_fit(rl_commands, timeline=300000)
    ll_command_bezier2 = WxWebotsApi.line_fit(ll_commands2,timeline=300000)
    rl1_is_over = False
    rl2_is_over = False
    rl3_is_over = False
    rh1_is_over = False
    rh2_is_over = False
    rh3_is_over = False
    ll_is_over = False

    '''
    step1：步态后退1步大小为0.3
    step2:调整右手手指头位置
    step3:抬起右手到冰箱门把手的位置
    step4:右手手指抓住把手
    step5:右手加力矩拉门把手，到达特定位置后取消力矩
    step6:左手抬起扶住冰箱门
    step7:松开右手手指
    step8：放下右手
    step9:放下左手
    '''
    if ws.walk_forward_step(step_num=1, linear_x=-0.30):
        while not rospy.is_shutdown():
            time.sleep(12)
            if not rh1_is_over:
                for i in range(1, 20000):
                    rh_controller(rh_names, rh_command1, mode=5)
                rh1_is_over = True
            elif not rl1_is_over:
                for cmd in rl_command_bezier:
                    rl_controller(rl_names, cmd, mode=5)
                rl1_is_over = True
            elif not rh2_is_over:
                for i in range(1, 50000):
                    rh_controller(rh_names, rh_command2, mode=5)
                rh2_is_over = True
            elif not rl2_is_over:
                first = rospy.get_time()
                last = rospy.get_time()
                rl_controller(rl_names, rl_command_force, mode=7)
                while (last - first) < 0.45:
                    last = rospy.get_time()
                rl_controller(rl_names, rl_command5, mode=5)
                rl2_is_over = True
            elif not ll_is_over:
                for cmd in ll_command_bezier:
                    ll_controller(ll_names, cmd, mode=5)
                ll_is_over = True
            elif not rh3_is_over:
                for i in range(1, 20000):
                    rh_controller(rh_names, rh_command3, mode=5)
                rh3_is_over = True
            elif not rl3_is_over:
                for cmd in rl_command_bezier2:
                    rl_controller(rl_names, cmd, mode=5)
                rl3_is_over = True
            else:
                for cmd in ll_command_bezier2:
                    ll_controller(ll_names, cmd, mode=5)
                break

    '''步态旋转'''
    # ws.start()
    # first = rospy.get_time()
    # last = rospy.get_time()
    # ws.vel(angular_z=0.3)
    # while (last - first)<2.0:
    #     ws.vel(angular_z=-0.2)
    #     last = rospy.get_time()
    # ws.stop()


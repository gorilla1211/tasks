import rospy
from utils.walker_scenes import scene_task10_OpenFridge
from utils.walker_func import WxWebotsApi
import time

wc = WxWebotsApi.WxWebotsStepFunc()
def task10_2():
    WxContrApi = WxWebotsApi.WxWebotsControllers()
    rl_controller = WxContrApi.publish_rightLimb_controller
    rh_controller = WxContrApi.publish_rightHand_controller
    scene_task10_OpenFridge()
    wc.start()
    step_flag = False
    first = rospy.get_time()
    last = rospy.get_time()
    while (last-first)<3.4:
        wc.vel(angular_z=-0.35)
        last = rospy.get_time()
    wc.stop()
    time.sleep(10)
    rl_names = ['RShoulderPitch', 'RShoulderRoll', 'RShoulderYaw',
        'RElbowRoll', 'RElbowYaw',
        'RWristPitch', 'RWristRoll']
    rl_command = [0 for _ in range(7)]
    rl1_is_over = False
    rl_command1 = [0, 0, 0, 0, 0, 0,0]
    rl_command2 = [-0.8, 0, 0, -0.5, 0, 0, 0]
    rl_command3 = [-1.2, 0, 0, -0.6, 0, 0, 0]
    rl_command4 = [-1.57,0.6, 0, -0.8, 0, 0, 0]
    rl_commands = [rl_command1, rl_command2, rl_command3, rl_command4]
    rl_command_bezier = WxWebotsApi.line_fit(rl_commands,timeline=1000000)
    while not rospy.is_shutdown():
        if not rl1_is_over:
            for cmd in rl_command_bezier:
                rl_controller(rl_names, cmd, mode=5)
            rl1_is_over = True
        else:
            break
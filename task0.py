"""
    初始化关节
"""
import rospy
from utils.walker_tops_srvs import BuildConnect, WalkerWebotsSub
from utils import WxWebotsApi
from utils.walker_types import JointCommand, JointState


def task0():
    WxCotroApi = WxWebotsApi.WxWebotsControllers()

    # 获得腿部关节控制器
    leg_DesiredJoint = WxCotroApi.publish_leg_DesiredJoint
    # 初始化腿部关节数据
    leg_DesiredJoint_data = JointState()
    leg_DesiredJoint_data.name = ['' for _ in range(12)] 
    leg_DesiredJoint_data.position= [0 for _ in range(12)]
    leg_DesiredJoint_data.velocity= [0 for _ in range(12)]
    leg_DesiredJoint_data.effort= [0 for _ in range(12)]
    # 初始化 position
    leg_DesiredJoint_data.position = [-2.5052541696618288e-08, -4.22196627124113e-05, -5.57557800606424e-05, -0.000157293707904186, -0.00011326930013588564, 3.595853411203041e-05,
                                      2.5291967815676514e-08, 4.274502435822778e-05, -5.5734891727255776e-05, -0.0001572456246685626, -0.00011323435588900862, -3.475137137467723e-05]
    
    # 获得左臂关节控制器
    leftLimb_controller = WxCotroApi.publish_leftLimb_controller
    # 初始化左臂关节数据
    leftLimb_controller_data = JointCommand()
    leftLimb_controller_data.names = ['' for _ in range(7)] 
    leftLimb_controller_data.command = [0 for _ in range(7)]
    leftLimb_controller_data.mode = 5
    # 初始化 command
    leftLimb_controller_data.command = [-2.7476229711559662e-05, 0.000548553138199625, 4.0090237114409884e-06,
                                        -0.0002707919077573215, 1.6172207976904167e-06,
                                        -1.6360476324464147e-05, 0.00010110325467476545]

    # 获得左手指关节控制器
    leftHand_controller = WxCotroApi.publish_leftHand_controller
    # 初始化左手指关节数据
    leftHand_controller_data = JointCommand()
    leftHand_controller_data.names = ['' for _ in range(10)] 
    leftHand_controller_data.command = [0 for _ in range(10)]
    leftHand_controller_data.mode = 5
    # 初始化 command
    leftHand_controller_data.command = [0.0005763275768218893, 0.00011048561576244416, -0.0002706140526310688, -0.0007861574955077551, -0.00028317286297405674,
                                        -0.0008233733154911486, -0.000421086441039068, -0.0011766369461278245, -0.00030181698272502863, -0.0012106571228496232]

    # 获得右臂关节控制器
    rightLimb_controller = WxCotroApi.publish_rightLimb_controller
    # 初始化右臂关节数据
    rightLimb_controller_data = JointCommand()
    rightLimb_controller_data.names = ['' for _ in range(7)] 
    rightLimb_controller_data.command = [0 for _ in range(7)]
    rightLimb_controller_data.mode = 5
    # 初始化 command
    rightLimb_controller_data.command = [2.6941435850886197e-05, 0.000546383420892756, -3.9183433139560056e-06,
                                         -0.0002692074006846251, -1.4736773997980114e-06,
                                         1.52698893558503e-05, 0.00010036750791670686]

    # 获得右手指关节控制器
    rightHand_controller = WxCotroApi.publish_rightHand_controller
    # 初始化右手指关节数据
    rightHand_controller_data = JointCommand()
    rightHand_controller_data.names = ['' for _ in range(10)] 
    rightHand_controller_data.command = [0 for _ in range(10)]
    rightHand_controller_data.mode = 5
    # 初始化 command
    rightHand_controller_data.command = [0.0007064327201528371, 0.0016695015266074394, -0.00024560449225025864, -0.0008262114036236796, -0.00024560442610779164,
                                         -0.0008262110810365624, -0.00043818052583673503, -0.0012361117895306238, -0.00031868718416866465, -0.0012732605290390732]

    print('开始初始化')
    count = 0
    # 全身关节初始化
    while not rospy.is_shutdown():
        count += 1
        leg_DesiredJoint(name=leg_DesiredJoint_data.name, position=leg_DesiredJoint_data.position,
                         velocity=leg_DesiredJoint_data.velocity)

        leftLimb_controller(names=leftLimb_controller_data.names, command=leftLimb_controller_data.command,
                            mode=leftLimb_controller_data.mode)
                            
        leftHand_controller(names=leftHand_controller_data.names, command=leftHand_controller_data.command,
                            mode=leftHand_controller_data.mode)

        rightLimb_controller(names=rightLimb_controller_data.names, command=rightLimb_controller_data.command,
                             mode=rightLimb_controller_data.mode)

        rightHand_controller(names=rightHand_controller_data.names, command=rightHand_controller_data.command,
                             mode=rightHand_controller_data.mode)
        
        if count == 10000:
            print('初始化完毕')
            count = 0
            break

"""
import necessary modules
"""
from dorna2 import pose as dorna_pose
import home
import home_config
import numpy as np

def start_event(msg, union, dorna_robot):
    if "in1" in msg and msg["in1"] == 1 and (dorna_robot.prm_stop or dorna_robot.prm_pause):
        dorna_robot.set_alarm(0)
        dorna_robot.prm_stop = False
        dorna_robot.prm_pause = False
        dorna_robot.prm_enable = True
        if dorna_robot.completed_sequence:
            dorna_robot.completed_sequence = False
            print("Starting new sequence")
        print("Start pressed, prm_stop: ", dorna_robot.prm_stop, " prm_pause: ", dorna_robot.prm_pause)

def stop_event(msg, union, dorna_robot):
    if "in2" in msg and msg["in2"] == 0 and not dorna_robot.prm_stop:
        dorna_robot.prm_stop = True
        dorna_robot.halt()
        dorna_robot.set_alarm(1)
        dorna_robot.prm_enable = False
        dorna_robot.completed_sequence = False
        if dorna_robot.sequence_started:
            dorna_robot.sequence_cancelled = True
        print("Stop pressed, prm_stop: ", dorna_robot.prm_stop)

def pause_event(msg, union, dorna_robot):
    if "in0" in msg and msg["in0"] == 0 and not dorna_robot.prm_pause:
        dorna_robot.prm_pause = True
        print("Pause pressed, prm_pause: ", dorna_robot.prm_pause)

def alarm_event(msg, union, dorna_robot):
    if "alarm" in msg and msg["alarm"] == 1 and not dorna_robot.prm_stop:
        dorna_robot.prm_stop = True
        dorna_robot.halt()
        dorna_robot.prm_enable = False
        dorna_robot.completed_sequence = False
        if dorna_robot.sequence_started:
            dorna_robot.sequence_cancelled = True
        print("Alarm activated, prm_stop: ", dorna_robot.prm_stop)

def update_robot_frame(config):
    config.robot["robot_x_world_off"] = config.robot["rail_screw_pos"] + config.robot["rail_screw_diameter"]/2 - config.robot["rail_scew_dist"] + config.robot["robot_base_x"] + config.robot["rail_x_ref"]
    config.robot["base_in_world"][0] = config.robot["robot_x_world_off"]

def update_vial_frame(config, vial_frame=[0,0], rail_x=485):
    config.vial_holder["frame"][0] = vial_frame[0]
    config.vial_holder["frame"][1] = vial_frame[1]
    config.vial_holder["aux"][0] = rail_x

    print("Vial in world position updated")

def update_cap_frame(config, cap_frame=[0,0], rail_x=485):
    config.cap_holder["frame"][0] = cap_frame[0]
    config.cap_holder["frame"][1] = cap_frame[1]
    config.cap_holder["aux"][0] = rail_x

    print("Cap in world position updated")

def output(config, robot, timeout=-1): 
    cmd_list = []   
    for c in config:
        cmd_list.append({"cmd": "output", "out" + str(c[0]): c[1], "queue": 0})
        if len(c) > 2 and c[2] > 0:
            cmd_list.append({"cmd": "sleep", "time": c[2], "queue": 0})
    return robot.play_list(cmd_list, timeout=timeout)

def init(config, robot, calibrate=False):
    # motor
    robot.set_motor(1)
    robot.sleep(0.25)

    #Update PIDs to increase crash sensitivity
    [org_threshold, org_duration, pid_factor] = [200, 10000, 1]

    for i in range(8):
        org_pid = robot.get_pid(i)
        robot.set_pid(i, org_pid[0], org_pid[1], org_pid[2], org_threshold*pid_factor, org_duration*pid_factor)

    #Lift the arm
    robot.sleep(0.25)
    acc_jrk = [1000, 8000]

    if config.sim == 0:
        robot.jmove(rel=0, sim=0, vel=150*config.speed, accel=acc_jrk[0], jerk=acc_jrk[1], j1=65, cont=0, timeout=-1)
        robot.sleep(0.5)
    
    #Calibrate the rail
    calibration_completed = False
    if calibrate and config.sim == 0:
        home_process = home.Home(robot, index=6, val=0, direction=-1, config=home_config.config["j"+ str(6)])
        rtn = home_process.start()
        home_process.end(rtn)
        calibration_completed = True
    
    if calibration_completed:
        print("Rail calibration completed")
    else:
        print("Calibration not performed")

    # Open grippers
    output(config.robot_gripper["open"], robot)
    output(config.decapper["open"], robot)
    output(config.cap_holder["open"], robot)
    output(config.capsule_opener["open"], robot)

    init_joints = config.robot["Init_joints"]+[config.robot["Init_aux"][0]]

    robot.jmove(rel=0,vel=150*config.speed,j0=init_joints[0],j1=init_joints[1],j2=init_joints[2],j3=init_joints[3],j4=init_joints[4],j5=init_joints[5],j6=init_joints[6])
    print("At initial position")

    # Signal robot is ready to continue:
    output(config.robot_gripper["close"], robot)
    output(config.robot_gripper["open"], robot)

def safe_init(config, robot):
    return robot.pick_n_place(**config.safe_init)

def vial_tray_to_decapper(config, robot):
    return robot.pick_n_place(**config.vial_tray_to_decapper)

def decapping(config, robot, pose_init=None):
    robot.sleep(0.25)

    if pose_init is None:
        pose_init = robot.pose()

    gripper_close = config.robot_gripper["close"]
    freedom = config.safe_init["freedom"]

    vaj = [600, 4000, 10000]
    if freedom == []:
        freedom = {"num":20, "range":[0.05, 0.05, 0.05], "early_exit":False}

    decapping_joints = [#With current setting, desired J5 middle point of decapping is -96°, this position alligns the hose fittings in the robot and in the gripper
        # [-52.185059, -12.612305, -47.15332, -0.021973, -30.300293, 220, 485],
        # [-52.185059, -11.66748, -48.405762, -0.021973, -30.036621, -220, 485]
        # [-52.163086, -12.875977, -46.252441, -0.021973, -30.9375, 220, 485], #Screw
        # [-52.163086, -11.931152, -47.526855, -0.021973, -30.651855, -220, 485] #Screw
        [-52.097168, -12.436523, -46.955566, 0, -30.60791, 144, 485], #Finger
        # [-52.097168, -11.491699, -48.186035, 0, -30.344238, -316, 485] #Finger
        [-52.097168, -11.645508, -47.988281, 0, -30.388184, -316, 485] # New finger
    ]

    for i in range(len(decapping_joints)):
        if i == 1:
            output(gripper_close, robot)
            robot.sleep(0.25)

        robot.jmove(rel=0, vel=vaj[0], cont=0, j0=decapping_joints[i][0], j1=decapping_joints[i][1], j2=decapping_joints[i][2], j3=decapping_joints[i][3], j4=decapping_joints[i][4], j5=decapping_joints[i][5], timeout=-1)

        # robot.go(joint=decapping_joints[i], vaj=vaj, motion="jmove", freedom=freedom)

    #Exit with the cap
    robot.lmove(rel=1, vel=150*config.speed, z=5, sim=config.sim)
    robot.jmove(rel=0, vel=vaj[0], j5=0, sim=config.sim)
    return

def decapper_to_cap_holder(config, robot):
    return robot.pick_n_place(**config.decapper_to_cap_holder)

def cap_to_vial(config, robot):
    return robot.pick_n_place(**config.cap_to_vial)

def capping(config, robot, pose_init=None):
    # #Original funtion:
    # if pose_init is None:
    #     pose_init = robot.pose()
    
    # gripper_open = config.robot_gripper["open"]
    # gripper_close = config.robot_gripper["close"]
    # freedom = config.cap_to_vial["freedom"]
    # round = config.bottle["round_cap"]
    # angle = config.bottle["angle_cap"]
    # prev_z_step = config.bottle["z_step_pre_cap"]
    # z_step = config.bottle["z_step_cap"]

    # vaj = [600, 4000, 10000]
    # if freedom == []:
    #     freedom = {"num":20, "range":[0.05, 0.05, 0.05], "early_exit":False}
    
    # robot.sleep(0.25) #Sleep to ensure the robot reached and stoped at its pre-decapping position
    # pose_init_prev = pose_init.copy()
    
    # pose_init_prev[3:6] = dorna_pose.rotate_abc(pose_init_prev[3:6], [0, 0, 1], 90, local=True)

    # for _ in range (2):
    #     # close
    #     output(gripper_close, robot)

    #     # pose end
    #     pose_init_prev[2] -= prev_z_step
    #     robot.go(pose=pose_init_prev, vaj=vaj, motion="lmove", freedom=freedom)
    #     robot.sleep(0.25)

    #     output(gripper_open, robot)

    #     # rotate back
    #     robot.lmove(rel=1, z=0.5)
    #     robot.jmove(rel=1, j5=-90, vel=vaj[0], accel=vaj[1], jerk=vaj[2])
    #     robot.lmove(rel=1, z=-0.5)

    # pose_init[3:6] = dorna_pose.rotate_abc(pose_init[3:6], [0, 0, 1], angle, local=True)
    # pose_init[2] -= prev_z_step * 2

    # for _ in range(round):
    #     # close
    #     output(gripper_close, robot)

    #     # pose end
    #     pose_init[2] -= z_step
    #     robot.go(pose=pose_init, vaj=vaj, motion="lmove", freedom=freedom)

    #     # open
    #     output(gripper_open, robot)

    #     # rotate back
    #     robot.lmove(rel=1, z=0.5)
    #     robot.jmove(rel=1, j5=-angle, vel=500, accel=10000, jerk=40000)
    #     robot.lmove(rel=1, z=-0.5)

    vaj = [700, 4000, 10000]
    #Decapping joints, should be used in reverse order for capping?:
    decapping_joints = [#With current setting, desired J5 middle point of decapping is -96°, this position alligns the hose fittings in the robot and in the gripper
        # [-52.097168, -12.436523, -46.955566, 0, -30.60791, 154, 485], #Before decapping
        [-52.141113, -12.348633, -47.06543, 0, -30.585938, 104, 485], #New Before decapping
        # [-52.097168, -11.491699, -48.186035, 0, -30.344238, -316, 485] #After decapping
        [-52.075195, -11.623535, -47.680664, -0.087891, -30.673828, -276, 485] #New After decapping
    ]
    robot.jmove(rel=0, vel=vaj[0], accel=vaj[1], jerk=vaj[2], cont=0, j0=decapping_joints[1][0], j1=decapping_joints[1][1], j2=decapping_joints[1][2], j3=decapping_joints[1][3], j4=decapping_joints[1][4], j5=decapping_joints[1][5], timeout=-1)
    robot.jmove(rel=0, vel=vaj[0], cont=0, j0=decapping_joints[0][0], j1=decapping_joints[0][1], j2=decapping_joints[0][2], j3=decapping_joints[0][3], j4=decapping_joints[0][4], j5=decapping_joints[0][5], timeout=-1)

    #J5 must return to 0 to avoid excesive motions. This motion also adds 90° of cap turn to tighten it further:
    output(config.robot_gripper["open"], robot)
    robot.jmove(rel=0, vel=vaj[0], cont=0, j5=-185, timeout=-1)
    output(config.robot_gripper["close"], robot)
    robot.jmove(rel=0, vel=vaj[0], cont=0, j5=-95, timeout=-1)
    output(config.decapper["open"], robot)
    robot.lmove(rel=1, vel=150*config.speed, cont=0, z=50, timeout=-1)

def vial_to_tray(config, robot):
    return robot.pick_n_place(**config.vial_to_tray)

def generate_vials_caps_coords():
    center_vial = [31*25+12.5, -(10*25+12.5)] #These are frame coordinates, calculated based on position on base plate
    Vials_coords = []
    [i,j] = [0,0] # i: row, j: column
    for _ in range(9):
        Vials_coords.append([center_vial[0] + (1-i)*14, center_vial[1] + (1-j)*14])
        j += 1
        if j == 3:
            j = 0
            i += 1

    center_cap = [37*25+12.5, -(6*25+12.5)] #These are frame coordinates, calculated based on position on base plate
    Caps_coords = []
    [i,j] = [0,0] # i: row, j: column
    for _ in range(9):
        Caps_coords.append([center_cap[0] + (1-i)*14, center_cap[1] + (1-j)*14])
        j += 1
        if j == 3:
            j = 0
            i += 1

    return Vials_coords, Caps_coords
"""
import necessary modules
"""
from dorna2 import pose as dorna_pose
import home
import home_config

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

def mix(config, robot):
    # Pending upgrade: Should take current position and calculate middle and end point based on that and radius
    # Initial: [234,0,160,179,0,0,492]
    # Middle: [244,-10,160,179,0,0,492]
    # Final: [254,0,160,179,0,0,492]
    r = config.mixer["r"]
    cmd = robot.cmove(rel=0,vel=220,turn=2,x=234+r*2,y=0,z=160,a=179,b=0,c=0,d=492,mx=234+r,my=-r,mz=160,ma=179,mb=0,mc=0,md=492)
    cmd = robot.cmove(rel=0,vel=220,turn=2,x=234,y=0,z=160,a=179,b=0,c=0,d=492,mx=234+r,my=-r,mz=160,ma=179,mb=0,mc=0,md=492)
    return cmd

def safe_init(config, robot):
    return robot.pick_n_place(**config.safe_init)

def vial_tray_to_decapper(config, robot):
    return robot.pick_n_place(**config.vial_tray_to_decapper)

def decapping(config, robot, pose_init=None):
    robot.sleep(0.5)

    if pose_init is None:
        pose_init = robot.pose()

    gripper_close = config.robot_gripper["close"]
    freedom = config.safe_init["freedom"]

    vaj = [600, 4000, 10000]
    if freedom == []:
        freedom = {"num":20, "range":[0.05, 0.05, 0.05], "early_exit":False}

    decapping_joints = [
        [-52.185059, -12.612305, -47.15332, -0.021973, -30.300293, 210, 485],
        [-52.185059, -11.513672, -48.60352, -0.021973, -29.992676, -210, 485]
    ]

    for i in range(len(decapping_joints)):
        if i == 1:
            output(gripper_close, robot)
            robot.sleep(0.25)

        robot.go(joint=decapping_joints[i], vaj=vaj, motion="jmove", freedom=freedom)

    #Exit with the cap
    robot.lmove(rel=1, vel=150*config.speed, z=5, sim=config.sim)
    return

def decapper_to_cap_holder(config, robot):
    return robot.pick_n_place(**config.decapper_to_cap_holder)

def cap_to_bottle(config, robot):
    return robot.pick_n_place(**config.cap_to_bottle)

def capping(config, robot, pose_init=None):
    if pose_init is None:
        pose_init = robot.pose()
    
    gripper_open = config.robot_gripper["open"]
    gripper_close = config.robot_gripper["close"]
    freedom = config.cap_to_bottle["freedom"]
    round = config.bottle["round_cap"]
    angle = config.bottle["angle_cap"]
    prev_z_step = config.bottle["z_step_pre_cap"]
    z_step = config.bottle["z_step_cap"]

    vaj = [600, 4000, 10000]
    if freedom == []:
        freedom = {"num":20, "range":[0.05, 0.05, 0.05], "early_exit":False}
        
    pose_init_prev = pose_init.copy()
    current_joint = robot.get_all_joint()
    print("pose_init_prev: ", pose_init_prev)
    print("current joints: ", current_joint)
    robot.sleep(1)
    
    pose_init_prev[3:6] = dorna_pose.rotate_abc(pose_init_prev[3:6], [0, 0, 1], 90, local=True)
    print("new pose_init_prev: ",pose_init_prev)

    for _ in range (2):
        # close
        output(gripper_close, robot)

        # pose end
        pose_init_prev[2] -= prev_z_step
        robot.go(pose=pose_init_prev, vaj=vaj, motion="lmove", freedom=freedom)
        # robot.lmove(rel=1, z=)
        robot.sleep(1)
        current_joint = robot.get_all_joint()
        print("current joints: ", current_joint)

        # open
        output(gripper_open, robot)

        # rotate back
        robot.lmove(rel=1, z=0.5)
        robot.jmove(rel=1, j5=-90, vel=vaj[0], accel=vaj[1], jerk=vaj[2])
        robot.lmove(rel=1, z=-0.5)

    pose_init[3:6] = dorna_pose.rotate_abc(pose_init[3:6], [0, 0, 1], angle, local=True)
    pose_init[2] -= prev_z_step * 2

    for _ in range(round):
        # close
        output(gripper_close, robot)

        # pose end
        pose_init[2] -= z_step
        robot.go(pose=pose_init, vaj=vaj, motion="lmove", freedom=freedom)

        # open
        output(gripper_open, robot)

        # rotate back
        robot.lmove(rel=1, z=0.5)
        robot.jmove(rel=1, j5=-angle, vel=500, accel=10000, jerk=40000)
        robot.lmove(rel=1, z=-0.5)

def bottle_to_tray(config, robot):
    return robot.pick_n_place(**config.bottle_to_tray)
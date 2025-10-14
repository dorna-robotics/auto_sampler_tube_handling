"""
import necessary modules
"""
from dorna2 import pose as dorna_pose
from dorna_vision import Detection
from dorna_vision import grasp
from dorna_vision import Valid
import home
import home_config
import numpy as np
import json
import time

def update_robot_frame(config):
    config.robot["robot_x_world_off"] = config.robot["rail_screw_pos"] + config.robot["rail_screw_diameter"]/2 - config.robot["rail_scew_dist"] + config.robot["robot_base_x"] + config.robot["rail_x_ref"]
    config.robot["base_in_world"][0] = config.robot["robot_x_world_off"]

def update_camera_frame(config, camera_frame):
    # config.camera["frame"][0] = camera_frame[0] + config.robot["base_in_world"][0] + config.camera["aux"][0]
    # config.camera["frame"][1] = camera_frame[1]
    config.camera["frame"][:2] = camera_frame[:2]
    print("Camera in world position updated")

# def update_vial_frame(config, vial_frame, rail_x=485):
#     config.vial_holder["frame"][0] = vial_frame[0]
#     config.vial_holder["frame"][1] = vial_frame[1]
#     config.vial_holder["aux"][0] = rail_x

#     print("Vial in world position updated")

def update_vial_frame_vision(config, vial_frame_vision, rail_x=485):
    # config.vial_holder["frame"][0] = vial_frame_vision[0] + config.robot["base_in_world"][0] + config.camera["aux"][0]
    # config.vial_holder["frame"][1] = vial_frame_vision[1]
    config.vial_holder["frame"][:2] = vial_frame_vision[:2]
    config.vial_holder["aux"][0] = rail_x

    # print("Vial frame: ", config.vial_holder["frame"], "Vial aux: ", config.vial_holder["aux"])
    print("Vial in world position updated")

def update_cap_frame(config, cap_frame, rail_x=485):
    config.cap_holder["frame"][0] = cap_frame[0]
    config.cap_holder["frame"][1] = cap_frame[1]
    config.cap_holder["aux"][0] = rail_x

    print("Cap in world position updated")

def correct_vision_xyz(config, vision_xyz, pxl, camera, camera_data, detection):
    z_guess = config.camera["z_guess"]
    z_gt = (z_guess, z_guess)
    xyz_target_to_cam = camera.xyz(pxl=pxl, depth_frame=camera_data["depth_frame"], depth_int=camera_data["depth_int"], z_gt=z_gt)

    T_target_to_cam = detection.kinematic.xyzabc_to_mat(np.concatenate((np.array(xyz_target_to_cam[0]), np.array([0, 0, 0]))))

    # apply frame
    T_target_to_frame = np.matmul(detection.frame_mat_inv, T_target_to_cam)
    xyz_target_to_frame = detection.kinematic.mat_to_xyzabc(T_target_to_frame).tolist()
    corrected_xyz = xyz_target_to_frame[0:3]
    return corrected_xyz

def translate_vis_to_world(config, xyz): #Naive translation, currently only corrects X as Y is already correct and this Z is disregarded by the pick_and_place motions
    corrected_xyz = xyz
    corrected_xyz[0] += config.robot["base_in_world"][0] + config.camera["aux"][0]
    return corrected_xyz

def cluster_coordinates(values, spacing, tolerance=0.6):
    """Cluster coordinates into grid positions"""
    sorted_vals = sorted(values)
    clusters = []
    current_cluster = [sorted_vals[0]]
    
    for val in sorted_vals[1:]:
        if val - current_cluster[-1] < spacing * tolerance:
            current_cluster.append(val)
        else:
            clusters.append(current_cluster)
            current_cluster = [val]
    clusters.append(current_cluster)
    
    # Return average of each cluster
    return [sum(c) / len(c) for c in clusters]

def assign_to_grid(point, row_centers, column_centers):
    x, y, z = point[0], point[1], point[2]
    
    # Find nearest row (based on X) and column (based on Y)
    row_idx = min(range(len(row_centers)), 
                key=lambda i: abs(row_centers[i] - x))
    col_idx = min(range(len(column_centers)), 
                key=lambda i: abs(column_centers[i] - y))
    
    return (row_idx, col_idx, point)

def sort_vials(config, points):
    grid_spacing = config.vial_holder["grid_spacing"]
    # Extract x and y coordinates
    x_coords = [p[0] for p in points]  # More positive = bottom, more negative = top
    y_coords = [p[1] for p in points]  # More positive = left, more negative = right

    # Find grid columns and rows (reference positions)
    # Sort x in reverse (top to bottom: most positive last)
    # Sort y in reverse (left to right: most positive first)
    column_centers = cluster_coordinates(y_coords, grid_spacing)  # Y defines columns (left-right)
    row_centers = cluster_coordinates(x_coords, grid_spacing)     # X defines rows (top-bottom)

    # Reverse row_centers so index 0 = top (most negative X)
    row_centers.reverse()
    # Reverse column_centers so index 0 = left (most positive Y)
    column_centers.reverse()

    print(f"Found {len(row_centers)} rows")
    print(f"Found {len(column_centers)} columns")

    # Assign all points to grid
    grid_points = [assign_to_grid(p, row_centers, column_centers) for p in points]

    # Sort by row (top to bottom), then by column (left to right)
    sorted_grid_points = sorted(grid_points, key=lambda x: (x[0], x[1]))

    # Extract just the points in the correct order
    sorted_points = [p[2] for p in sorted_grid_points]

    # print("\nSorted points (left to right, top to bottom):")
    # for i, point in enumerate(sorted_points, 1):
    #     print(f"{i}. {point}")
    return sorted_points

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

def take_init_picture(config, robot):
    joints = config.camera["joints"] + config.camera["aux"]
    robot.go(joint=joints, motion="jmove", freedom=config.take_init_picture["freedom"], cont=0, timeout=-1)
    # return robot.pick_n_place(**config.take_init_picture)

def take_ind_picture(config, robot):
    return robot.pick_n_place(**config.take_ind_picture)

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
    capping_joints = [#With current setting, desired J5 middle point of decapping is -96°, this position alligns the hose fittings in the robot and in the gripper
        # [-52.075195, -11.623535, -47.680664, -0.087891, -30.673828, -276, 485], #Before capping
        # [-52.141113, -12.348633, -47.06543, 0, -30.585938, 104, 485], #After capping
        [-51.723633, -12.414551, -45.769043, 0, -31.772461, -301, 485], #Before capping
        [-51.745605, -13.249512, -44.714355, 0, -32.01416, 129, 485], #After capping
    ]
    robot.jmove(rel=0, vel=vaj[0], accel=vaj[1], jerk=vaj[2], cont=0, j0=capping_joints[0][0], j1=capping_joints[0][1], j2=capping_joints[0][2], j3=capping_joints[0][3], j4=capping_joints[0][4], j5=capping_joints[0][5], timeout=-1)
    robot.jmove(rel=0, vel=vaj[0], cont=0, j0=capping_joints[1][0], j1=capping_joints[1][1], j2=capping_joints[1][2], j3=capping_joints[1][3], j4=capping_joints[1][4], j5=capping_joints[1][5], timeout=-1)

    #J5 must return to 0 to avoid excesive motions. This motion also adds 90° of cap turn to tighten it further:
    output(config.robot_gripper["open"], robot)
    robot.jmove(rel=0, vel=vaj[0], cont=0, j5=-185, timeout=-1)
    output(config.robot_gripper["close"], robot)
    robot.jmove(rel=0, vel=vaj[0], cont=0, j5=-95, timeout=-1)
    output(config.decapper["open"], robot)
    robot.lmove(rel=1, vel=150*config.speed, cont=0, z=50, timeout=-1)

def vial_to_tray(config, robot):
    return robot.pick_n_place(**config.vial_to_tray)

def vial_tray_to_decapper_vision(config, robot, detection):
    retval = None

    # imaging joint
    robot.go(joint=config.camera["joints"]+ config.tube_pick["aux"], speed=config.speed, sim=0)
    time.sleep(0.5)

    # run detection
    result_tube = detection.run()
    for r in result_tube:
        # not a cap
        if r["cls"] not in config.robot_gripper:
            continue
        
        # valid xyz
        if not Valid().xyz(r["xyz"], **config.detection_preset["limit"]["xyz"]):
            continue

        # best_pick
        best_rvec_pick = grasp.collision_free_rvec(
            r["id"], 
            config.two_finger_gripper[r["cls"]]["rvec_base"], 
            config.two_finger_gripper[r["cls"]]["gripper_opening"],
            config.two_finger_gripper[r["cls"]]["finger_width"],
            config.two_finger_gripper[r["cls"]]["finger_location"], 
            detection, mask_type="elp", prune_factor=4, num_steps=360)        
        if best_rvec_pick is None:
            print("no grasp found")
            continue
        
        # all the possible solutions
        pose_all = [r["xyz"] + dorna_pose.rotate_abc(best_rvec_pick, axis=rotation["axis"], angle=rotation["angle"], local=True) for rotation in config.two_finger_gripper[r["cls"]]["gripper_rotation"]]

        # nearest pose
        best_pose = robot.kinematic.nearest_pose(pose_all, detection.retval["camera_data"]["joint"], config.rack_to_scanner["freedom"])
        if best_pose is None:
            print("no nearest pose found")
            continue
        
        # adjust pick_n_place
        config.rack_to_scanner["pick"]["loc"][0] = best_pose
        config.rack_to_scanner["pick"]["tool"] = 2*[config.two_finger_gripper[r["cls"]]["tool"]]
        config.rack_to_scanner["pick"]["output"] = config.two_finger_gripper[r["cls"]]["close"]
        config.rack_to_scanner["place"]["tool"] = 2*[config.two_finger_gripper[r["cls"]]["tool"]]
        
        # rack_to_scanner
        result = robot.pick_n_place(**config.rack_to_scanner)
        if not result:
            print("pick and place failed")
            continue
        try:
            for res in result:
                print(json.dumps(res))
        except:
            pass
        
        retval = r["cls"], best_pose
        break
    
    return retval

def generate_vials_caps_coords():
    # center_vial = [31*25+12.5, -(10*25+12.5)] #These are frame coordinates, calculated based on position on base plate
    center_vial = [31*25+12.5-(1.4+12.5)/2, -(10*25+12.5)+(1.4+12.5)/2] #These are frame coordinates, calculated based on position on base plate
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
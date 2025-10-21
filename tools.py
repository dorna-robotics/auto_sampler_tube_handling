"""
import necessary modules
"""
from dorna2 import pose as dorna_pose
from dorna_vision import Detection
from dorna_vision import grasp
from dorna_vision import Valid
from scipy.interpolate import griddata
import home
import home_config
import numpy as np
import json
import time

def update_robot_frame(config):
    config.robot["robot_x_world_off"] = config.robot["rail_screw_pos"] + config.robot["rail_screw_diameter"]/2 - config.robot["rail_scew_dist"] + config.robot["robot_base_x"] + config.robot["rail_x_ref"]
    config.robot["base_in_world"][0] = config.robot["robot_x_world_off"]

def update_camera_frame(config, camera_frame):
    config.camera["frame"][:2] = camera_frame[:2]

    print("Camera in world position updated")

def update_vial_frame_vision(config, vial_frame_vision, rail_x=None):
    if rail_x is None:
        rail_x = config.camera["aux"][0]
    config.vial_holder["frame"][:2] = vial_frame_vision[:2]
    config.vial_holder["aux"][0] = rail_x

    print("Vial in world position updated")

def update_cap_frame(config, cap_frame, rail_x=485):
    config.cap_holder["frame"][0] = cap_frame[0]
    config.cap_holder["frame"][1] = cap_frame[1]
    config.cap_holder["aux"][0] = rail_x

    print("Cap in world position updated")

def correct_vision_xyz(config, pxl, camera, camera_data, detection):
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
    # x, y, z = point[0], point[1], point[2]
    x, y = point[0], point[1]
    
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

# def take_ind_picture(config, robot):
#     return robot.pick_n_place(**config.take_ind_picture)

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
        [-52.097168, -12.436523, -46.955566, 0, -30.60791, 144, 435],
        [-52.097168, -11.645508, -47.988281, 0, -30.388184, -316, 435]
    ]

    for i in range(len(decapping_joints)):
        if i == 1:
            output(gripper_close, robot)
            robot.sleep(0.25)

        robot.jmove(rel=0, vel=vaj[0], cont=0, j0=decapping_joints[i][0], j1=decapping_joints[i][1], j2=decapping_joints[i][2], j3=decapping_joints[i][3], j4=decapping_joints[i][4], j5=decapping_joints[i][5], timeout=-1)

    #Exit with the cap
    robot.lmove(rel=1, vel=150*config.speed, z=5, sim=config.sim)
    robot.jmove(rel=0, vel=vaj[0], j5=0, sim=config.sim)
    return

def decapper_to_cap_holder(config, robot):
    return robot.pick_n_place(**config.decapper_to_cap_holder)

def cap_to_vial(config, robot):
    return robot.pick_n_place(**config.cap_to_vial)

def capping(config, robot, pose_init=None):
    vaj = [700, 4000, 10000]
    # rail = config.decapper["aux"][0] #Could be used to ensure rail has the right value based directly on previous motion
    # capping_joints = [#With current setting, desired J5 middle point of decapping is -96°, this position alligns the hose fittings in the robot and in the gripper
    #     [-51.723633, -12.414551, -45.769043, 0, -31.772461, -301, 435], #Before capping
    #     [-51.745605, -13.249512, -44.714355, 0, -32.01416, 129, 435], #After capping
    # ]

    capping_joints = [#With current setting, desired J5 middle point of decapping is -96°, this position alligns the hose fittings in the robot and in the gripper
        [-51.723633, -12.590332, -45.505371, 0, -31.904297, -53, 435], #Before first capping turn
        [-51.723633, -12.963867, -45.021973, 0, -32.01416, 307, 435], #After first capping turn
        [-51.723633, -12.766113, -45.263672, 0, -31.948242, -275, 435], #Before second capping turn
        [-51.723633, -12.963867, -45.021973, 0, -32.01416, -95, 435], #After second capping turn
    ]
    robot.jmove(rel=0, vel=vaj[0], accel=vaj[1], jerk=vaj[2], cont=0, j0=capping_joints[0][0], j1=capping_joints[0][1], j2=capping_joints[0][2], j3=capping_joints[0][3], j4=capping_joints[0][4], j5=capping_joints[0][5], timeout=-1)
    robot.jmove(rel=0, vel=vaj[0], cont=0, j0=capping_joints[1][0], j1=capping_joints[1][1], j2=capping_joints[1][2], j3=capping_joints[1][3], j4=capping_joints[1][4], j5=capping_joints[1][5], timeout=-1)

    #J5 must return to 0 to avoid excesive motions. This motion also adds 90° of cap turn to tighten it further:
    output(config.robot_gripper["open"], robot)
    # robot.jmove(rel=0, vel=vaj[0], cont=0, j5=-185, timeout=-1)
    robot.jmove(rel=0, vel=vaj[0], accel=vaj[1], jerk=vaj[2], cont=0, j0=capping_joints[2][0], j1=capping_joints[2][1], j2=capping_joints[2][2], j3=capping_joints[2][3], j4=capping_joints[2][4], j5=capping_joints[2][5], timeout=-1)
    output(config.robot_gripper["close"], robot)
    # robot.jmove(rel=0, vel=vaj[0], cont=0, j5=-95, timeout=-1)
    robot.jmove(rel=0, vel=vaj[0], accel=vaj[1], jerk=vaj[2], cont=0, j0=capping_joints[3][0], j1=capping_joints[3][1], j2=capping_joints[3][2], j3=capping_joints[3][3], j4=capping_joints[3][4], j5=capping_joints[3][5], timeout=-1)
    output(config.decapper["open"], robot)
    robot.lmove(rel=1, vel=150*config.speed, cont=0, z=50, timeout=-1)

def vial_to_tray(config, robot):
    return robot.pick_n_place(**config.vial_to_tray)

def generate_caps_coords():
    center_cap = [35*25+12.5, -(6*25+12.5)] #These are frame coordinates, calculated based on position on base plate
    Caps_coords = []
    [i,j] = [0,0] # i: row, j: column
    for _ in range(9):
        Caps_coords.append([center_cap[0] + (1-i)*14, center_cap[1] + (1-j)*14])
        j += 1
        if j == 3:
            j = 0
            i += 1

    return Caps_coords

def generate_vial_coords_vision(config, vial_vis_pxls):
    #Calibration data:
    # cal_pxl_positions = [
    #     #Visual:
    #     [234, 343], #Top left
    #     [290, 157], #Top center
    #     [390, 25], #Top right
    #     [307, 394], #Center left
    #     [422, 236], #Center
    #     [506, 63], #Center right
    #     [450, 463], #Bottom left
    #     [540, 319], #Bottom center
    #     [622, 161] #Bottom right
    # ]

    # cal_robot_coords = [
    #     #Visual X and Y:
    #     [184.465494, -210.403642], #Top left
    #     [192.734647, -270.087779], #Top center
    #     [182.474894, -319.387575], #Top right
    #     [157.988354, -206.496997], #Center left
    #     [145.443447, -265.152446], #Center
    #     [145.251345, -324.158048], #Center right
    #     [107.502585, -204.368494], #Bottom left
    #     [101.743223, -257.788362], #Bottom center
    #     [100.219337, -313.212557] #Bottom right
    # ]

    # cal_pxl_positions = [
    #     #Visual:
    #     [234, 343], #Top left
    #     [302, 168], #Top center
    #     [353, 22], #Top right
    #     [373, 439], #Center left
    #     [389, 241], #Center
    #     [470, 46], #Center right
    #     [511, 466], #Bottom left
    #     [579, 271], #Bottom center
    #     [643, 103] #Bottom right
    # ]

    # cal_robot_coords = [
    #     #Visual X and Y:
    #     [136.414375, -184.90726], #Top left
    #     [137.484441, -247.979733], #Top center
    #     [135.884311, -296.912878], #Top right
    #     [90.72823, -174.934899], #Center left
    #     [104.737793, -235.944598], #Center
    #     [97.959783, -299.930801], #Center right
    #     [47.466675, -178.941335], #Bottom left
    #     [47.002161, -244.990474], #Bottom center
    #     [43.982242, -298.850564] #Bottom right
    # ]

    cal_pxl_positions = [
        #Visual:
        [239, 367], #Top left
        [301, 178], #Top center
        [350, 30], #Top right
        [381, 433], #Center left
        [403, 214], #Center
        [456, 57], #Center right
        [494, 458], #Bottom left
        [564, 259], #Bottom center
        [616, 112] #Bottom right
    ]

    cal_robot_coords = [
        #Visual X and Y:
        [137.967645, -182.933609], #Top left
        [136.992617, -245.923724], #Top center
        [135.006638, -293.912369], #Top right
        [88.93142, -177.962435], #Center left
        [102.954943, -245.845898], #Center
        [100.936002, -294.872737], #Center right
        [52.944641, -179.908295], #Bottom left
        [51.944164, -246.934241], #Bottom center
        [49.936064, -294.98378] #Bottom right
    ]

    x_comp = config.robot["base_in_world"][0] + config.camera["aux"][0]

    cal_world_coords = []
    for x in cal_robot_coords:
        coord = x
        coord[0] += x_comp
        cal_world_coords.append(coord)

    Vial_grid_coords = []

    for x in vial_vis_pxls:
        result_griddata = griddata(cal_pxl_positions, cal_world_coords, x, method='cubic')

        Vial_grid_coords.append(result_griddata[0].tolist())

    return Vial_grid_coords

#############################################################
# Robot
#############################################################
robot = {
    "ip": "192.168.254.104",
    "frame_in_world": [0, 0, 0, 0, 0, 0],
    "base_in_world": [115+5*25+12.5, 0, 82.25, 0, 0, 0], # Shouldn't be called base_in_frame?
    "Init_joints": [-60.007324, 22.390137, -92.460938, 0, -19.929199, -60.007324], #Equivalent to: [288, 0.05, 102.82, 179.99, 0.04, 0.03] in cartesian
    "Init_aux": [485,0],
    "aux_dir": [[1, 0, 0], [0, 0, 0]],
    "rail_screw_pos": 2 * 25 + 12.5, # Position of the limit screw in world frame. Fixed, only depends on which base plate hole the screw is at
    "rail_screw_diameter": 4.95, # Real diameter of the limit screw. Measured from real one
    "rail_scew_dist": 37.25, # Distance between the scew and the robot's base rear edge. Measured when rail_x = 0
    "rail_x_ref": 0, # rail_scew_dist was measured when rail_x = 0
    "robot_base_x": 130, # Distance between j0 axis and the robot's base rear edge. Fixed, depends only on the robot's dimensions. Was taken from the 3D CAD, not measured from real robot
    "robot_x_world_off": 0, # This is the position in the world frame of the robot's origin when the rail is at rail_x = 0
    "mode": "dorna_ta"
}

#############################################################
# Motion control and sim
#############################################################
sim = 0
speed = 1
sleep = 0
num = 25

#############################################################
# Items
#############################################################
bottle = {
    "height":90,
    "round_cap":6, #8
    "angle_cap": 90, #90
    "z_step_cap": 0.3, #0.5
    "z_step_pre_cap": 0.3 #0.1
}

cap = {
    "height":12,
}

#############################################################
# Grippers
#############################################################
robot_gripper = {
    "cap_decap": [0, 0, 12+27+6.9, 0, 0, 0], #Z=screw out of gripper+gripper+gripper base, this gives the lowest point of the screws
    "open": [[1, 0, 0.25]],
    "close": [[1, 1, 0.25]],
}

#############################################################
# Stations
#############################################################
decapper = {
    "aux": [485, 0],
    "frame": [37*25+12.5-23.5, -(10*25+12.5), 0, 180, 0, 0],
    "vial_drop": [-0.13949975311390972, 4.105025002541481, -39.18238253652287, 0.23170989712236975, -0.33843512546358934, 48.040617793171805],
    "vial_remove_cap": [-4.2066696461535, 3.6970004445179825, -43.452655907016194, 0.0016795849423070833, -0.024713694437646778, 96.09716670016483], #Done before robot recalibration, no longer valid
    "vial_pick": [-4.602693987670477, 4.337848920734132, -35.5493322734435, 0.23452653920962876, -0.3244143124774287, 48.01046749832838],
    "cap_drop": [-0.2850402522691411, 4.727967870527436, -41.64823677029854, -0.010522056052978799, 0.02003404402905445, 48.03222639721679],
    "r": 3, #Radius used to shake the capsule on top of the bottle after pouring in the powder
    "open": [[0, 0, 0.25]],
    "close": [[0, 1, 0.25]]
}

vial_holder = {
    "aux": [485, 0],
    "frame": [31*25+12.5-(1.4+12.5)/2, -(10*25+12.5)+(1.4+12.5)/2, 0, 180, 0, 0], #Position of vial at col 3, row 4 from tray's bottom left corner
    "pick": [-0.05716812026093976, 1.1853255529723015, -33.06382287695952, 0.23287556085751854, -0.3172811067209202, 47.98701517310126],
    "drop": [-0.1455158211095977, 1.2360057567397007, -37.077112148668476, 0.2327585229236095, -0.3173253109692344, 48.00894415631942]
}

cap_holder = {
    "aux": [485, 0],
    "frame": [37*25+12.5, -(6*25+12.5), 0, 180, 0, 0],
    "pick": [0.5286622981476512, 3.5193017123831964, -10.858378489849372, -4.769162562411459e-16, 2.572475798769063e-15, -42.011718],
    "drop": [-3.1190497933401957, 3.1627060997030867, -13.228152233190258, 0.02347517880592545, -0.08709918072320023, 45.15379995820485] #Done before robot recalibration, no longer valid
}

#############################################################
# Motions
#############################################################
safe_init = {
    "pick": {
        "type": "joint", # robot, world, joint
        "loc": [robot["Init_joints"], robot["Init_aux"]],
        "output": robot_gripper["open"] + decapper["open"]
    },
    "base_in_world": robot["base_in_world"],
    "aux_dir": robot["aux_dir"],
    "sleep": sleep, 
    "motion": "lmove", "speed": speed, "cont": 0, "corner": 250, 
    "freedom": {"num":num, "range":[0.05, 0.05, 0.05], "early_exit":False}, "timeout": -1, "sim":sim
}

vial_tray_to_decapper = {
    "pick": {
        "type": "world", # robot, world, joint
        "loc": [vial_holder["pick"], vial_holder["aux"]],
        "frame": vial_holder["frame"],
        "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]], #[0]: Tool defiinition before grabbing, [1]: Tool after grabbing
        "output": robot_gripper["close"] + decapper["open"], # List of lists where each element has the format: [pin_index,state,sleep]
        "approach": [[0, 0, -50, 0, 0, 0]],
        "exit": [[0, 0, -50, 0, 0, 0]]
    },
    "place": {
        "type": "world", # robot, world, joint
        "loc": [decapper["vial_drop"], decapper["aux"]],
        "frame": decapper["frame"],
        "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]],
        "output": robot_gripper["open"] + decapper["close"], # List of lists where each element has the format: [pin_index,state,sleep]
        "approach": [[0, 0, -50, 0, 0, 0],[0, 0, -24, 0, 0, 0]],
        # "exit": [[0, 0, 2, 0, 0, 0]] #For original full sequence
        "exit": [[0, 0, -10, 0, 0, 0]] #For new sequence, capping and placing only
    },
    "base_in_world": robot["base_in_world"],
    "aux_dir": robot["aux_dir"],
    "sleep": sleep, 
    "motion": "lmove", "speed": speed, "cont": 0, "corner": 250, 
    "freedom": {"num":num, "range":[0.05, 0.05, 0.05], "early_exit":False}, "timeout": -1, "sim":sim
}

decapper_to_cap_holder = {
    "pick": {
        "type": "world", # robot, world, joint
        "loc": [decapper["vial_remove_cap"], decapper["aux"]],
        "frame": decapper["frame"],
        "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]],
        "exit": [[0, 0, -20, 0, 0, 0]]
    },
    "place": {
        "type": "world", # robot, world, joint
        "loc": [cap_holder["drop"], cap_holder["aux"]],
        "frame": cap_holder["frame"],
        "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]],
        "output": robot_gripper["open"], # List of lists where each element has the format: [pin_index,state,sleep]
        "approach": [[0, 0, -20, 0, 0, 0],[0, 0, -3, 0, 0, 0]],
        "exit": [[0, 0, -10, 0, 0, 0]]
    },
    "base_in_world": robot["base_in_world"],
    "aux_dir": robot["aux_dir"],
    "sleep": sleep, 
    "motion": "lmove", "speed": speed, "cont": 0, "corner": 250, 
    "freedom": {"num":num, "range":[0.05, 0.05, 0.05], "early_exit":False}, "timeout": -1, "sim":sim
}

cap_to_vial = {
    "pick": {
        "type": "world", # robot, world, joint
        "loc": [cap_holder["pick"], cap_holder["aux"]],
        "frame": cap_holder["frame"],
        "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]], #[0]: Tool defiinition before grabbing, [1]: Tool after grabbing
        "output": robot_gripper["close"] + decapper["close"], # List of lists where each element has the format: [pin_index,state,sleep]
        "approach": [[0, 0, -30, 0, 0, 0]],
        "exit": [[0, 0, -40, 0, 0, 0]]
    },
    "place": {
        "type": "world", # robot, world, joint
        "loc": [decapper["cap_drop"], decapper["aux"]],
        "frame": decapper["frame"],
        "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]],
        # "approach": [[0, 0, -24, 0, 0, 0]]
    },
    "base_in_world": robot["base_in_world"],
    "aux_dir": robot["aux_dir"],
    "sleep": sleep, 
    "motion": "lmove", "speed": speed, "cont": 0, "corner": 250, 
    "freedom": {"num":num, "range":[0.05, 0.05, 0.05], "early_exit":False}, "timeout": -1, "sim":sim
}

vial_to_tray = {
    "pick": {
    #     "type": "world", # robot, world, joint
    #     "loc": [decapper["vial_pick"], decapper["aux"]],
    #     "frame": decapper["frame"],
    #     "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]],
    #     "output": decapper["open"] + robot_gripper["close"], # List of lists where each element has the format: [pin_index,state,sleep]
    #     "exit": [[0, 0, -50, 0, 0, 0]]
    # },
    # "place": {
        "type": "world", # robot, world, joint
        "loc": [vial_holder["drop"], vial_holder["aux"]],
        "frame": vial_holder["frame"],
        "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]], #[0]: Tool defiinition before grabbing, [1]: Tool after grabbing
        "output": robot_gripper["open"], # List of lists where each element has the format: [pin_index,state,sleep]
        "approach": [[0, 0, -70, 0, 0, 0],[0, 0, -24.5, 0, 0, 0],[0, 0, -14, 0, 0, 0]], #Safe vertical align, just above the other vials, just above the hole entrance, just clear of the bottom
        "exit": [[0, 0, -50, 0, 0, 0]]
    },
    "base_in_world": robot["base_in_world"],
    "aux_dir": robot["aux_dir"],
    "sleep": sleep, 
    "motion": "lmove", "speed": speed, "cont": 0, "corner": 250, 
    "freedom": {"num":num, "range":[0.05, 0.05, 0.05], "early_exit":True}, "timeout": -1, "sim":sim
}
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
    # "vial_drop": [-3.4922009622124506, 2.9801683921267568, -36.84284411822889, 0.0, 0.0, 0.0], #Screw
    "vial_drop": [-4.1191069531492985, 3.783898190173659, -37.53536285318124, 0.009932736598999443, 0.0056356552051534305, 45.08486363011147], #Finger
    # "vial_drop": [-4.390686292410692, 3.979806058651775, -33.8856069457848, 0.02434753910013784, -0.08989178250944486, 45.138061153420544],
    # "vial_remove_cap": [-1.7522009622124506, 4.2301683921267568, -43.84284411822889, 0.0, 0.0, 0.0], #Screw
    "vial_remove_cap": [-4.2066696461535, 3.6970004445179825, -43.452655907016194, 0.0016795849423070833, -0.024713694437646778, 96.09716670016483], #Finger
    # "vial_pick": [-3.4922009622124506, 2.9801683921267568, -35.84284411822889, 0.0, 0.0, 0.0], #Screw
    "vial_pick": [-3.551885315868617, 4.123022456699346, -35.2555004881015, 0.044251724469281176, 0.07130399902402108, -43.17609634441307], #Finger
    # "cap_drop": [-3.8922009622124506, 2.9801683921267568, -38.00284411822889, 0.0, 0.0, 0.0], #Original
    # "cap_drop": [-3.8922009622124506, 2.9801683921267568, -37.50284411822889, 0.0, 0.0, 0.0], #Screw
    "cap_drop": [-4.169256283574782, 3.7450551245523798, -48.45038120564836, -1.4062032671192587e-06, -1.4062032812152803e-06, -107.902832], #New finger, aligned with initial capping pose
    "r": 3, #Radius used to shake the capsule on top of the bottle after pouring in the powder
    "open": [[0, 0, 0.25]],
    "close": [[0, 1, 0.25]]
}

vial_holder = {
    "aux": [485, 0],
    "frame": [31*25+12.5, -(10*25+12.5), 0, 180, 0, 0],
    # "pick": [-(12.5+1.4)/2, -(12.5+1.4)/2, 0, 0, 0, 0],
    # "pick": [-9.764187893421422, -7.412270400775412, -32.36704425351625, -0.021326592825661406, 0.005505911820115738, 0.021955259660476633], #Screw
    "pick": [-11.162325540724964, -6.777140928163249, -32.44861340291981, 0.0009670873896716081, -0.0003938777206854976, 45.13183018233267], #Finger
    # "drop": [-9.764187893421422, -7.412270400775412, -34.36704425351625, -0.021326592825661406, 0.005505911820115738, 0.021955259660476633] #Screw
    "drop": [-10.690417492448773, -7.091159268335161, -36.70595196203988, -4.708816203908032e-16, 2.5563306064601616e-15, -41.74804699999998] #Finger
}

cap_holder = {
    "aux": [485, 0],
    "frame": [37*25+12.5, -(6*25+12.5), 0, 180, 0, 0],
    # "pick": [-2.595807898578528, 2.9618301041078894, -3.760334578530404-9.5, 0.0, 0.0, 0.0], #Original
    # "pick": [-3.595807898578528, 2.9618301041078894, -3.760334578530404-9.5, 0.0, 0.0, 0.0], #New screw
    "pick": [-3.20817908999868, 2.4275495226249575, -10.713640711747558, 0.040508790759103006, 0.06856503003917214, -43.1650801910818], # New finger
    # -10.713640711747558
    # "drop": [-4.565807898578528, 3.9618301041078894, -4.060334578530404-9.5, 0.0, 0.0, 0.0] #Original
    # "drop": [-4.565807898578528, 3.9618301041078894, -3.860334578530404-9.5, 0.0, 0.0, 0.0] #New Screw
    # "drop": [-3.1190497933401957, 3.1627060997030867, -13.228152233190258, 0.040508790759103006, 0.06856503003917214, -43.1650801910818] #Finger original
    "drop": [-3.1190497933401957, 3.1627060997030867, -13.228152233190258, 0.02347517880592545, -0.08709918072320023, 45.15379995820485] #Finger, original position with new orientation
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
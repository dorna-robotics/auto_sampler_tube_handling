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
    "vial_drop": [0, 0, 71, 0, 0, 0],
    "open": [[1, 0, 0.25]],
    "close": [[1, 1, 0.25]],
    "vial":{
        "tool": [0, 0, 29, 0, 0, 0],
        "close": [[0, 1, 0.25]],
        "open": [[0, 0, 0.25]],
        "finger_width": 10,
        "gripper_opening": 43,
        "finger_location": [0, 180],
        "rvec_base": [180, 0, 0],
        "gripper_rotation": [
            {"axis": [0, 0, 1], "angle": 0},
            {"axis": [0, 0, 1], "angle": 180},
        ]
    }
}

#############################################################
# Stations
#############################################################
decapper = {
    "aux": [435, 0],
    "frame": [35*25+12.5-23.5, -(10*25+12.5), 0, 180, 0, 0],
    "vial_drop": [-0.13949975311390972, 4.105025002541481, -39.18238253652287, 0.23170989712236975, -0.33843512546358934, 48.040617793171805],
    "vial_remove_cap": [-4.2066696461535, 3.6970004445179825, -43.452655907016194, 0.0016795849423070833, -0.024713694437646778, 96.09716670016483], #Done before robot recalibration, no longer valid
    "vial_pick": [-4.602693987670477, 4.337848920734132, -35.5493322734435, 0.23452653920962876, -0.3244143124774287, 48.01046749832838],
    "cap_drop": [-0.2850402522691411, 4.727967870527436, -41.64823677029854, -0.010522056052978799, 0.02003404402905445, 48.03222639721679],
    "open": [[0, 0, 0.25]],
    "close": [[0, 1, 0.25]]
}

vial_holder = {
    "aux": [485, 0], #Original
    "frame": [778.2555219769545, -270.90092983538375, 0, 180, 0, 0], #This exact value is not relevant, it's updated on each iteration before pick up
    "pick": [1.2848547803481551, -0.6376452704425901, -33.06382287695952, 0.23287556085751854, -0.3172811067209202, 47.98701517310126],
    "vial_insertion": [0.7780088287922808, 1.5867262079533475, -25.5959426972259, -6.694248991944859, 2.7723314835873385, 44.72395062043049],
    "drop": [1.2848547803481551, -0.6376452704425901, -14.334862947201302, -1.2390190108418174e-06, 1.5174821696505201e-06, 45.000000000000014],
    "grid_spacing": 12.5 + 1.4,
    "vial_tilt": 30 #Tilt angle for the vials to ensure easy insertion in tray. Applied to globla X and Y axes, not done locally
}

cap_holder = {
    "aux": [435, 0],
    "frame": [35*25+12.5, -(6*25+12.5), 0, 180, 0, 0],
    "pick": [0.5286622981476512, 3.5193017123831964, -10.858378489849372, -4.769162562411459e-16, 2.572475798769063e-15, -42.011718],
    "drop": [-3.1190497933401957, 3.1627060997030867, -13.228152233190258, 0.02347517880592545, -0.08709918072320023, 45.15379995820485] #Done before robot recalibration, no longer valid
}

camera = {
    # "aux": [435, 0], #Old
    "aux": [485, 0],
    "frame": [31*25+12.5, -(10*25+12.5), 0, 180, 0, 0],
    "vials": [-0.8261485518240761, -52.573060583650374, -84.59651760709431, 0.020111097002960723, -0.00885000707069195, 1.5043170460124506e-13], #Calculated from "frame" and "joints" using gripper["cap_decap"]
    # "joints": [-64.907227, 6.569824, -109.138184, 0.021973, 12.546387, -64.929199], #Old
    "joints": [-72.817383, 1.867676, -120.498047, 0.021973, 28.608398, -72.817383], #New
    "ind_vial": [4.215628633946608, -55.61708449389786, -84.58102105223001, 0.017875335293693777, -0.013641431814762501, -0.0005246204567250441],
    "z_guess": 141.5
}

#############################################################
# detections
#############################################################
detection_preset = {
    "tube":{
        "camera_mount":{
            "type": "dorna_ta_j4",
            "ej": [0 ,0, 0, 0, 0, 0, 0, 0],
            "T": [46.5174596+1+1+0+4-(5+0.75), 32.0776662-3+1-0-1.5+(-3+2.2), -4.24772615-3, -0.27547989, 0.27691881, 89.6939516],
        },
        "detection": {"cmd": "od", "path": "model/tube.pkl", "conf": 0.2, "cls": []},
        #'roi': {'corners': [[214.51, 11.63], [220.24, 377.26], [611.69, 374.4], [613.13, 10.19]], 'inv': False, 'crop': True, 'offset': 0},
        "roi": {"corners": [[652.99, 428.85], [675.93, 7.29], [174.08, 2.98], [179.81, 437.45]], "inv": False, "crop": True, "offset": 0},
        "sort": {"cmd": "shuffle", "max_det": 100}, 
        #'limit': {'xyz': {'x': [-150, 110], 'y': [200, 450], 'z': [110, 130], 'inv': 0}}, 
        "display": {"label": 0, "save_img": False, "save_img_roi": True}
        },
    "limit": {"xyz": {"x": [-150, 110], "y": [200, 450], "z": [110, 130], "inv": 0}}, 
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

take_init_picture = {
    "pick": {
        "type": "joint", # robot, world, joint
        "loc": [camera["joints"], camera["aux"]]
    },
    "base_in_world": robot["base_in_world"],
    "aux_dir": robot["aux_dir"],
    "sleep": sleep, 
    "motion": "jmove", "speed": speed, "cont": 0, "corner": 250, 
    "freedom": {"num":num, "range":[0.05, 0.05, 0.05], "early_exit":False}, "timeout": -1, "sim":0
}

# take_ind_picture = {
#     "pick": {
#         "type": "world", # robot, world, joint
#         "loc": [camera["ind_vial"], camera["aux"]],
#         "frame": camera["frame"],
#         "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]], #[0]: Tool defiinition before grabbing, [1]: Tool after grabbing
#     },
#     "base_in_world": robot["base_in_world"],
#     "aux_dir": robot["aux_dir"],
#     "sleep": sleep, 
#     "motion": "jmove", "speed": speed, "cont": 0, "corner": 250, 
#     "freedom": {"num":num, "range":[0.05, 0.05, 0.05], "early_exit":False}, "timeout": -1, "sim":0
# }

vial_tray_to_decapper = {
    "pick": {
        "type": "world", # robot, world, joint
        "loc": [vial_holder["pick"], vial_holder["aux"]],
        "frame": vial_holder["frame"],
        "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]], #[0]: Tool defiinition before grabbing, [1]: Tool after grabbing
        "output": robot_gripper["close"] + decapper["open"], # List of lists where each element has the format: [pin_index,state,sleep]
        "approach": [[0, 0, -40, 0, 0, 0]],
        "exit": [[0, 0, -50, 0, 0, 0]]
    },
    "place": {
        "type": "world", # robot, world, joint
        "loc": [decapper["vial_drop"], decapper["aux"]],
        "frame": decapper["frame"],
        "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]],
        "output": robot_gripper["open"] + decapper["close"], # List of lists where each element has the format: [pin_index,state,sleep]
        "approach": [[0, 0, -50, 0, 0, 0],[0, 0, -24, 0, 0, 0]],
        "exit": [[0, 0, -10, 0, 0, 0]]
    },
    "base_in_world": robot["base_in_world"],
    "aux_dir": robot["aux_dir"],
    "sleep": sleep, 
    "motion": "lmove", "speed": speed, "cont": 0, "corner": 250, 
    "freedom": {"num":num, "range":[0.1, 0.1, 0.1], "early_exit":False}, "timeout": -1, "sim":sim
}

# decapper_to_cap_holder = {
#     "pick": {
#         "type": "world", # robot, world, joint
#         "loc": [decapper["vial_remove_cap"], decapper["aux"]],
#         "frame": decapper["frame"],
#         "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]],
#         "exit": [[0, 0, -20, 0, 0, 0]]
#     },
#     "place": {
#         "type": "world", # robot, world, joint
#         "loc": [cap_holder["drop"], cap_holder["aux"]],
#         "frame": cap_holder["frame"],
#         "tool": [robot_gripper["cap_decap"], robot_gripper["cap_decap"]],
#         "output": robot_gripper["open"], # List of lists where each element has the format: [pin_index,state,sleep]
#         "approach": [[0, 0, -20, 0, 0, 0],[0, 0, -3, 0, 0, 0]],
#         "exit": [[0, 0, -10, 0, 0, 0]]
#     },
#     "base_in_world": robot["base_in_world"],
#     "aux_dir": robot["aux_dir"],
#     "sleep": sleep, 
#     "motion": "lmove", "speed": speed, "cont": 0, "corner": 250, 
#     "freedom": {"num":num, "range":[0.05, 0.05, 0.05], "early_exit":False}, "timeout": -1, "sim":sim
# }

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
        "approach": [[0, 0, -20, 0, 0, 0]]
    },
    "base_in_world": robot["base_in_world"],
    "aux_dir": robot["aux_dir"],
    "sleep": sleep, 
    "motion": "lmove", "speed": speed, "cont": 0, "corner": 250, 
    "freedom": {"num":num, "range":[0.05, 0.05, 0.05], "early_exit":False}, "timeout": -1, "sim":sim
}

vial_to_tray = {
    "pick": {
        "type": "world", # robot, world, joint
        "loc": [vial_holder["drop"], vial_holder["aux"]],
        "frame": vial_holder["frame"],
        "tool": [robot_gripper["vial_drop"], robot_gripper["vial_drop"]], #[0]: Tool defiinition before grabbing, [1]: Tool after grabbing
        "output": robot_gripper["open"], # List of lists where each element has the format: [pin_index,state,sleep]
        "approach": [[0, 0, -60, 0, 0, 0],[0, 0, -12, 0, vial_holder["vial_tilt"], 0],[0, 0, -9, 0, 0, 0]], #Safe vertical align, just above the hole entrance, just clear of the bottom
        "exit": [[0, 0, -50, 0, 0, 0]]
    },
    "base_in_world": robot["base_in_world"],
    "aux_dir": robot["aux_dir"],
    "sleep": sleep, 
    "motion": "lmove", "speed": speed, "cont": 0, "corner": 250, 
    "freedom": {"num":num, "range":[0.05, 0.05, 0.05], "early_exit":True}, "timeout": -1, "sim":sim
}
from dorna2 import Dorna
import tools
import config
import time

robot = Dorna()
robot_connected = False
connection_try_counter = 0

while not robot_connected:
    if robot.connect(config.robot["ip"]):
        print("Robot connected")
        robot_connected = True
    else:
        print("Robot connection failed")

    if not robot_connected:
        connection_try_counter += 1
        time.sleep(1)
        if connection_try_counter == 100:
            print("Maximum connection attempts reached")

# Initialize control variables
# Calculate the position of the robot's x offset in the world frame
tools.update_robot_frame(config=config)

tools.safe_init(config=config, robot=robot)

#Generate target coordinates:
Vials_coords, Caps_coords = tools.generate_vials_caps_coords()

print("Init completed")

vials_to_do = 9
ind = 0

for i in range(vials_to_do):
    if i < ind:
        continue
    elif i > ind:
        ind = i

    start_time = time.time()
    tools.update_vial_frame(config=config, vial_frame=Vials_coords[ind])
    tools.update_cap_frame(config=config, cap_frame=Caps_coords[ind])

    #Original demo sequence:
    # tools.vial_tray_to_decapper(config=config, robot=robot)
    # tools.decapping(config=config, robot=robot)

    # tools.decapper_to_cap_holder(config=config, robot=robot)

    # tools.cap_to_vial(config=config, robot=robot)
    # tools.capping(config=config, robot=robot)
    # tools.vial_to_tray(config=config, robot=robot)

    #New sequence:
    tools.vial_tray_to_decapper(config=config, robot=robot)
    tools.cap_to_vial(config=config, robot=robot)
    tools.capping(config=config, robot=robot)
    tools.vial_to_tray(config=config, robot=robot)
    end_time = time.time()

    print("Vial processing completed")
    print("Pick and place movement execution time: ", end_time-start_time)

print("Sequence completed")

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

print("Init completed")

# Initialize control variables
# Calculate the position of the robot's x offset in the world frame
tools.update_robot_frame(config=config)

tools.safe_init(config=config, robot=robot)
tools.vial_tray_to_decapper(config=config, robot=robot)
tools.decapping(config=config, robot=robot)
robot.sleep(10)
tools.decapper_to_cap_holder(config=config, robot=robot)

robot.sleep(10)

tools.cap_to_bottle(config=config, robot=robot)
tools.capping(config=config, robot=robot)
tools.bottle_to_tray(config=config, robot=robot)

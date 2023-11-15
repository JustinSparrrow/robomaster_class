import time
import robomaster
from robomaster import robot


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta")

    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm
    ep_chassis = ep_robot.chassis

    ep_arm.recenter() # 机械臂回中

    ep_gripper.open(power=50)   # 张开机械爪
    time.sleep(1)
    ep_gripper.pause()

    ep_chassis.move(x=0.9, y=0, z=0, xy_speed=0.9).wait_for_completed() # 底盘向前运动0.9m

    ep_arm.move(x=100, y=0).wait_for_completed()    # 机械臂向前移动0.1m
    ep_arm.move(x=0, y=-100).wait_for_completed()   # 机械臂向下移动0.1m

    ep_gripper.close(power=50)  # 闭合机械爪
    time.sleep(1)
    ep_gripper.pause()

    ep_chassis.move(x=-0.9, y=0, z=0, xy_speed=0.9).wait_for_completed()    # 地盘向后运动0.9m

    ep_arm.move(x=0, y=100).wait_for_completed()    # 机械臂向上移动0.1m
    ep_arm.move(x=-100, y=0).wait_for_completed()   # 机械臂向后移动0.1m

    ep_robot.close()

#
# Copyright (C) 2019  DENSO WAVE INCORPORATED
#
# -*- coding: utf-8 -*-
#
# usage: python ./packing_pose.py
#
#!/usr/bin/env python
import os
import sys
import rospy
import actionlib
import math
import moveit_commander
import rosservice
from geometry_msgs.msg import Pose, Point, Quaternion
from denso_cobotta_gripper.msg import GripperMoveAction, GripperMoveGoal
from denso_cobotta_driver.srv import GetMotorState

# NOTE: Before start this program, please launch denso_cobotta_bring.launch

joints_name = ["joint_1", "joint_2",
               "joint_3", "joint_4", "joint_5", "joint_6"]

#
# Poses
#
joints_packing_0 = [0, 0, 30, 0, 0, 0]
joints_packing_1 = [90, 0, 30, 0, 0, 0]
joints_packing_2 = [90, 30, 30, 0, 0, 0]
joints_packing_3 = [90, 30, 45, 0, 0, 0]
joints_packing_4 = [90, 30, 45, 90, 0, 0]
joints_packing_5 = [90, 30, 45, 90, 90, 0]
joints_packing_6 = [90, 30, 45, 90, 90, 90]

#
# Parallel gripper
#
gripper_parallel_open = 0.015
gripper_parallel_close = 0
gripper_parallel_speed = 10.0
gripper_parallel_effort = 10.0

def arm_move(move_group, joint_goal):
    pose_radian = [x / 180.0 * math.pi for x in joint_goal]
    move_group.go(pose_radian, wait=True)
    move_group.stop()


def gripper_move(gripper_client, width, speed, effort):
    goal = GripperMoveGoal()
    goal.target_position = width
    goal.speed = speed
    goal.effort = effort
    gripper_client.send_goal(goal)


def is_motor_running():
    rospy.wait_for_service('/cobotta/get_motor_state', 3.0)
    try:
        get_motor_state = rospy.ServiceProxy('/cobotta/get_motor_state',
                                             GetMotorState)
        res = get_motor_state()
        return res.state
    except rospy.ServiceException, e:
        print >> sys.stderr, "  Service call failed: %s" % e

def is_simulation():
    service_list = rosservice.get_service_list()
    if '/cobotta/get_motor_state' in service_list:
        return False
    return True

if __name__ == '__main__':
    rospy.init_node("packing_pose")
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("arm")
    gripper_client = actionlib.SimpleActionClient('/cobotta/gripper_move',
                                                  GripperMoveAction)


    print(os.path.basename(__file__) + " sets pose goal and moves COBOTTA.")
    print("0: reset pose, 1: joint1, 2: joint2, 3: joint3, 4: joint4, 5: joint5, 6: joint6, Others: Exit")
    while True:
        input = raw_input("  Select the value: ")
        if input.isdigit():
            input = int(input)

        joints = []
        gripper_width = 0.0

        if input == 0:
            joints = joints_packing_0
            gripper_width = gripper_parallel_open
        elif input == 1:
            joints = joints_packing_1
            gripper_width = gripper_parallel_close
        elif input == 2:
            joints = joints_packing_2
            gripper_width = gripper_parallel_close
        elif input == 3:
            joints = joints_packing_3
            gripper_width = gripper_parallel_close
        elif input == 4:
            joints = joints_packing_4
            gripper_width = gripper_parallel_close
        elif input == 5:
            joints = joints_packing_5
            gripper_width = gripper_parallel_close
        elif input == 6:
            joints = joints_packing_6
            gripper_width = gripper_parallel_close
        elif input == 7:
            joints = joints_packing_6
            gripper_width = gripper_parallel_open
        else:
            break

        if not is_simulation() and is_motor_running() is not True:
            print >> sys.stderr, "  Please motor on."
            continue

        gripper_move(gripper_client, gripper_width,
                     gripper_parallel_speed, gripper_parallel_effort)
        arm_move(move_group, joints)

    print("Bye...")

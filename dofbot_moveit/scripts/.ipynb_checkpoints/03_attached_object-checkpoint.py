#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
from time import sleep
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface

if __name__ == "__main__":
    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)
    # 初始化ROS节点
    rospy.init_node('dofbot_attached_object_py')
    # 初始化场景对象
    scene = PlanningSceneInterface()
    # 初始化需要使用move group控制的机械臂中的arm group
    dofbot = MoveGroupCommander('dofbot')
    # 设置"up"为目标点
    dofbot.set_named_target("up")
    dofbot.go()
    sleep(0.5)
    # 设置桌面的高度
    table_ground = 0.2
    # 设置障碍物的三维尺寸[长宽高]
    table_size = [0.7, 0.1, 0.02]
    # 将table加入场景当中
    table_pose = PoseStamped()
    table_pose.header.frame_id = 'base_link'
    table_pose.pose.position.x = 0
    table_pose.pose.position.y = 0.15
    table_pose.pose.position.z = table_ground + table_size[2] / 2.0
    table_pose.pose.orientation.w = 1.0
    scene.add_box('table', table_pose, table_size)
    rospy.sleep(2)
    dofbot.set_named_target("down")
    dofbot.go()
    sleep(0.5)
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


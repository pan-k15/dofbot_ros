# !/usr/bin/env python
# coding: utf-8
from time import sleep
import rospy
from moveit_commander.move_group import MoveGroupCommander

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node("dofbot_set_move")
    # 初始化机械臂
    dofbot = MoveGroupCommander("dofbot")
    while (1):
        # 设置随机目标点
        dofbot.set_random_target()
        # 开始运动
        dofbot.go()
        sleep(0.5)
        # 设置"up"为目标点
        dofbot.set_named_target("up")
        dofbot.go()
        sleep(0.5)
        # 设置"down"为目标点
        dofbot.set_named_target("down")
        dofbot.go()
        sleep(0.5)

#!/usr/bin/env python
import rospy
import json
import numpy as np
import time

from libs.kinematics import Kinematics
from config.config import Config

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy


N = 5

HOME_POSE = [0, 0, 0, 0, 0]

INIT_XYZ = [-0.03856489,  0.00063685,  0.47336637]
INIT_RPY = [-0.029632687787158475, 0.34859823089929853, 3.0385202467828036]
# -0.03856489  0.00063685  0.47336637 -0.029632687787158475 0.34859823089929853 3.0385202467828036
CONSTR_MIN_QS = [0.1, 0.1, -5.08, 0.1, 0.1]
CONSTR_MAX_QS = [5.8, 2.6, -0.1, 3.48, 5.75]

SCALE = 0.01

class JoyCart:
    """Joy control in Cartesian space"""

    def __init__(self, dev):
        self.config = {"btn": {"goHome": 0, "goInit": 1}, "axe": {"cY": 6, "cX": 5, "rX": 2, "rY": 3, "lX": 0, "lY": 1}}
        self.numOfAx = -1

        self.curPos = [[-0.03856489,  0.00063685,  0.47336637], [0, 0, 0]]
        self.prevCurPos = [[-0.03856489,  0.00063685,  0.47336637], [-0.029632687787158475, 0.34859823089929853, 3.0385202467828036]]
        self.prevQ = [0,0,0,0,0]
        self.pubJointState = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.ks = Kinematics(Config.DH_A, Config.DH_ALPHA, Config.DH_D, Config.DH_THETA)
        self.loop()

    def resetJoy(self):
        """Reset to initial pos"""
        self.curPos[0] = INIT_XYZ
        self.curPos[1] = INIT_RPY

    def goHome(self):
        pass

    def isConvenient(self, qs):
        """Check for joint's constraints"""
        for j in range(0, 4):
            flag = True
            rospy.loginfo(CONSTR_MIN_QS)
            rospy.loginfo(qs[j])
            rospy.loginfo(CONSTR_MAX_QS)

            for i in range(N):
                if qs[j][i] <= CONSTR_MIN_QS[i] + 0.1:
                    print('OOOPS_NO CONVENIENT')
                    flag = False
                if qs[j][i] >= CONSTR_MAX_QS[i] - 0.1:
                    print('OOOPS_NO CONVENIENT')
                    flag = False
            if flag:
                return j, True
        return -1, False

    def updatePose(self, xyz, rpy):
        # rospy.loginfo(xyz)
        # rospy.loginfo(rpy)

        for i in range(3):
            self.curPos[0][i] += xyz[i] * SCALE
            self.curPos[1][i] += rpy[i] * SCALE

    def goToPose(self, q):
        jointState = JointState()
        jointState.header.stamp = rospy.Time.now()
        jointState.name = ['arm_joint_' + str(i + 1) for i in range(5)]
        jointState.name.extend(['gripper_finger_joint_l', 'gripper_finger_joint_r'])
        jointState.position = q.tolist()
        # add positions for fingers of gripper
        jointState.position.extend([0.015, 0.015])
        self.pubJointState.publish(jointState)

    def joyCallback(self, joyMsg):
        # x = joyMsg.axes[self.config['t']['x']]
        # y = joyMsg.axes[self.config['t']['y']]
        # z = joyMsg.axes[self.config['t']['z']]
        # roll = joyMsg.axes[self.config['r']['roll']]
        # pitch = joyMsg.axes[self.config['r']['pitch']]
        x = joyMsg.axes[3]
        y = joyMsg.axes[2]
        z = joyMsg.axes[1]
        roll = 0
        pitch = joyMsg.axes[5]
        yaw = joyMsg.axes[0]
        if(joyMsg.buttons[0]):
            self.resetJoy()

        self.updatePose((x, y, z), (roll, pitch, yaw))
        qs, soluts = self.ks.inverse(self.curPos[0], self.curPos[1])
        num, condition = self.isConvenient(qs)
        if soluts[0] and soluts[1] and condition:
            q = qs[num]
            self.prevQ = q
            self.goToPose(q)
        else:
            self.goToPose(self.prevQ)


    def setup(self):
        """Configure gamepad's controls"""
        try:
            fileConfig = open("config/default_joy.config", 'r')
            config = json.load(fileConfig)
        except:
            print("Sorry, but I cant find default configs! Bey-Bey!\n")
            rospy.signal_shutdown("No config!")
            return None
        return config

    def loop(self):
        rospy.init_node('joy_controller')
        sub = rospy.Subscriber("joy", Joy, self.joyCallback)
        # self.config = self.setup()
        self.resetJoy()
        rospy.spin()

if __name__=="__main__":
    jsDev = "/dev/input/js1"
    JoyCart(jsDev)

#!/usr/bin/env python
import rospy
import json
import os
import time

from sensor_msgs.msg import Joy

DEFAULT_FILE_CONFIG = 'config/default_joy.json'

# needness buttons
# l -- left, r -- right, c -- cross
USE_PIPS = ['lX', 'lY', 'rX', 'rY', 'goHome', 'initPose']


class JoySetup:
    """May be useful for configuring your control device"""

    def __init__(self):
        sub = rospy.Subscriber('joy', Joy, self.joyCallback)

        self.touchedAxle = -42
        self.prevAxle = []
        self.touchedButton = -42

        self.loop()

    def joyCallback(self, jmsg):
        # check if touched any of axes or buttons
        # and set it number to class variable
        for numAx, ax in enumerate(jmsg.axes[:6]):
            if ax > 0:
                self.touchedAxle = numAx
                break
            else:
                self.touchedAxle = -42

        for numBtn, btn in enumerate(jmsg.buttons):
            if btn > 0:
                self.touchedButton = numBtn
                break
            else:
                self.touchedButton = -42

    def setup(self):
        config = {'btn': {}, 'axe': {}}
        answer = raw_input('Do you ready to configure you control device? [y/n] ')
        if answer in ['y', 'Y']:
            retry = 'n'
            while retry not in ['y', 'Y']:
                cfgMsg = "Touch {0} and FREEZE for one second!!!1"
                for pip in USE_PIPS:
                    print(cfgMsg.format(pip))

                    flag = True
                    while flag:
                        if self.touchedButton != -42:
                            config['btn'][pip] = self.touchedButton
                            flag = False
                        elif self.touchedAxle != -42:
                            config['axe'][pip] = self.touchedAxle
                            flag = False
                        time.sleep(0.1)
                    print('Ok, release all buttons!')
                    time.sleep(2)
                print("Your config is:\n{0}".format(config))
                retry = raw_input("It's ok??7? [y/n] ")
            fileName = raw_input("Print config file name??7? [new_joy.json] ")
            if fileName != '':
                fileConfig = open('config/' + fileName, 'w')
            else:
                fileConfig = open('config/new_joy.json', 'w')
            json.dump(config, fileConfig)
        else:
            if os.path.exists(DEFAULT_FILE_CONFIG):
                fileConfig = open(DEFAULT_FILE_CONFIG, 'r')
                default_config = json.load(fileConfig)
                print("Your default config:\n{}".format(str(default_config)))
            else:
                print('Oo-o-ops! No config file1!')
        print('Good luck!')
        rospy.signal_shutdown('Mission compete!!1')

    def loop(self):
        rospy.init_node('joy_setup')
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rospy.logdebug("Node start!")
            self.setup()
            rate.sleep()

if __name__ == '__main__':
    JoySetup()

#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
import subprocess

def play_sound(req):
    rospy.loginfo("Play!")
    rc = -1
    while(rc != 0):
        p = subprocess.Popen(["aplay","/catkin_ws/Calibration is r 1.wav"], stdin= subprocess.PIPE, stdout= subprocess.PIPE, stderr= subprocess.PIPE)
        output, err = p.communicate(b"")
        rc = p.returncode
        #print(rc)
    return EmptyResponse()

def run():
    rospy.init_node("play_sound")

    s = rospy.Service("~play", Empty, play_sound)

    rospy.spin()

if __name__ == '__main__':
    run()

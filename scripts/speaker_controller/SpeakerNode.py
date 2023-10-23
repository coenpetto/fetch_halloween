#!/usr/bin/env python

# rosdep install sound_play
# rosmake sound_play

import sys
import os
from sound_yak.msg import yak_cmd
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest
import rospy
import roslib
roslib.load_manifest('sound_yak')

# directory with sound assets
soundAssets = '/home/catkin_ws/src/fetch-halloween/sounds/'
# duration of yak throttle
throttle = 3  # seconds


def sound_translator(data):
    print(data)
    global allow_yak
    if rospy.Time.now() <= allow_yak:  # Throttles yak to avoid
        print("Sound throttled")      # SoundClient segfault
        return
    # when to reallow yak
    allow_yak = rospy.Time.now() + rospy.Duration.from_sec(throttle)
    if data.cmd == "wav":
        soundhandle.playWave(soundAssets + data.param)
    if data.cmd == "say":
        soundhandle.say(data.param)


def yak_init():
    rospy.init_node('yak_node', anonymous=True)
    global allow_yak
    allow_yak = rospy.Time.now()
    rospy.Subscriber('yak', yak_cmd, sound_translator)
    rospy.spin()


if __name__ == '__main__':
    soundhandle = SoundClient()
    rospy.sleep(1)

    yak_init()

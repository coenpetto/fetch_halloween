#!/usr/bin/env python

import rospy
import random
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient


sound_assets = '/home/bot_ws/src/fetch-halloween/sounds/'
sounds = ['audio1.wav', 'audio2.wav', 'audio3.wav', 'audio4.wav',
          'audio5.wav', 'audio6.wav', 'audio7.wav', 'audio8.wav', 'audio9.wav']

if __name__ == "__main__":
    # Create a node
    rospy.init_node("test_sound")

    soundhandle = SoundClient()

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    soundhandle.playWave(
        sound_assets + random.choice(sounds))

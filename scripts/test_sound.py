#!/usr/bin/env python

import rospy
import random
from playsound import playsound


sound_assets = '/home/fetch/bot_ws/src/fetch-halloween/sounds/'
sounds = ['audio1.wav', 'audio2.wav', 'audio3.wav', 'audio4.wav',
          'audio5.wav', 'audio6.wav', 'audio7.wav', 'audio8.wav', 'audio9.wav']

if __name__ == "__main__":
    # Create a node
    rospy.init_node("test_sound")

    # Wait for subscribers
    rospy.sleep(1)

    # Create and populate the SoundRequest message
    playsound(sound_assets + random.choice(sounds))

    # Give the sound time to play (optional: depends on your application's requirements)
    rospy.sleep(5)

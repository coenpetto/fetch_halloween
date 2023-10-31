#!/usr/bin/env python

import rospy
import random
from sound_play.msg import SoundRequest

sound_assets = '/home/bot_ws/src/fetch-halloween/sounds/'
sounds = ['audio1.wav', 'audio2.wav', 'audio3.wav', 'audio4.wav',
          'audio5.wav', 'audio6.wav', 'audio7.wav', 'audio8.wav', 'audio9.wav']

if __name__ == "__main__":
    # Create a node
    rospy.init_node("test_sound")

    # Create a publisher for the SoundRequest message
    pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=1)

    # Wait for subscribers
    rospy.sleep(1)

    # Create and populate the SoundRequest message
    sound_request = SoundRequest()
    sound_request.sound = SoundRequest.PLAY_FILE
    sound_request.command = SoundRequest.PLAY_ONCE
    sound_request.arg = sound_assets + random.choice(sounds)

    # Publish the sound request message
    pub.publish(sound_request)

    # Give the sound time to play (optional: depends on your application's requirements)
    rospy.sleep(5)

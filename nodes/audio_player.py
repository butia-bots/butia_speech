#!/usr/bin/env python3

import os
import rospy

from butia_speech.wav_to_mouth import WavToMouth
from butia_speech.srv import AudioPlayer, AudioPlayerResponse

from termcolor import colored
import warnings
warning = rospy.get_param("warnings", False)
if not warning:
    warnings.filterwarnings("ignore")

def toTalk(req):
    filepath = req.audio_path
    
    wm = WavToMouth(filepath=filepath)
    wm.read_audio()

if __name__ == "__main__":
    rospy.init_node("audio_player", anonymous=False)

    audio_player_service_param = rospy.get_param("services/audio_player/service", "/butia_speech/ap/audio_player")
    rospy.Service(audio_player_service_param, AudioPlayer, toTalk)

    print(colored("Audio Player is on!", "green"))
    rospy.spin()
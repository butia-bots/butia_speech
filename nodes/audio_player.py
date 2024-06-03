#!/usr/bin/env python3

import os
import rospy

from butia_speech.wav_to_mouth import WavToMouth
from butia_speech.srv import AudioPlayer, AudioPlayerResponse, AudioPlayerByData, AudioPlayerByDataResponse
from std_msgs.msg import Bool

from termcolor import colored
import warnings
warning = rospy.get_param("warnings", False)
warnings.filterwarnings("ignore")

wm = None

def toTalk(req):
    filepath = req.audio_path
    
    wm.set_filepath(filepath)
    wm.play_all_data()

    return AudioPlayerResponse(Bool(True))

def toTalkByData(req):
    data = req.data.data
    info = req.audio_info

    wm = WavToMouth()
    wm.set_data_and_info(data, info)
    wm.play_all_data()

    return AudioPlayerByDataResponse(Bool(True))

if __name__ == "__main__":
    rospy.init_node("audio_player", anonymous=False)

    wm = WavToMouth()

    audio_player_service_param = rospy.get_param("services/audio_player/service", "/butia_speech/ap/audio_player")
    rospy.Service(audio_player_service_param, AudioPlayer, toTalk)

    audio_player_by_data_service_param = rospy.get_param("services/audio_player_by_data/service", "/butia_speech/ap/audio_player_by_data")
    rospy.Service(audio_player_by_data_service_param, AudioPlayerByData, toTalkByData)

    print(colored("Audio Player is on!", "green"))
    rospy.spin()
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
    
    wm.set_filepath(filepath)
    wm.play_all_data()

    return AudioPlayerResponse(True)
        
def toTalkByData(req):
    if wm.streaming:
        return AudioPlayerByDataResponse(False)
    data = req.data.data
    info = req.audio_info

    wm.set_data_and_info(data, info)
    wm.play_all_data()

    return AudioPlayerByDataResponse(True)

def audioStreamStart(req):
    global last_stream_data_timestamp
    if wm.streaming:
        return AudioStreamStartResponse(False)
    
    last_stream_data_timestamp = rospy.get_rostime()
    
    info = req.audio_info
    wm.set_audio_info(info)

    wm.start_stream()

    return AudioStreamStartResponse(True)

def stop_stream(req):
    global last_stream_data_timestamp
    wm.request_stop_stream()

    last_stream_data_timestamp = None

    while wm.streaming:
        time.sleep(0.1)

    return EmptyResponse()

def stream_data_callback(data):
    global last_stream_data_timestamp
    if wm.streaming:
        last_stream_data_timestamp = rospy.get_rostime()
        wm.stream_data_callback(data.data)

if __name__ == "__main__":
    rospy.init_node("audio_player", anonymous=False)

    audio_player_service_param = rospy.get_param("services/audio_player/service", "/butia_speech/ap/audio_player")
    rospy.Service(audio_player_service_param, AudioPlayer, toTalk)

    print(colored("Audio Player is on!", "green"))
    rospy.spin()
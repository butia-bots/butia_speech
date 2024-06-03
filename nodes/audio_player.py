#!/usr/bin/env python3

import os
import rospy
import time

from butia_speech.wav_to_mouth import WavToMouth
from butia_speech.srv import AudioPlayer, AudioPlayerResponse, AudioPlayerByData, AudioPlayerByDataResponse, AudioStreamStart, AudioStreamStartResponse
from audio_common_msgs.msg import AudioData
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Bool

from termcolor import colored
import warnings
warning = rospy.get_param("warnings", False)
warnings.filterwarnings("ignore")

wm = None

def toTalk(req):
    if wm.streaming:
        return AudioPlayerResponse(Bool(False))

    filepath = req.audio_path
    
    wm.set_filepath(filepath)
    wm.play_all_data()

    return AudioPlayerResponse(Bool(True))
        
def toTalkByData(req):
    if wm.streaming:
        return AudioPlayerByDataResponse(Bool(False))
    data = req.data.data
    info = req.audio_info

    wm.set_data_and_info(data, info)
    wm.play_all_data()

    return AudioPlayerByDataResponse(Bool(True))

def audioStreamStart(req):
    if wm.streaming:
        return AudioStreamStartResponse(Bool(False))
    
    info = req.audio_info
    wm.set_audio_info(info)

    wm.start_stream()

    return AudioStreamStartResponse(Bool(True))

def stop_stream(req):
    wm.request_stop_stream()

    while wm.streaming:
        time.sleep(0.1)

    return EmptyResponse()

def stream_data_callback(data):
    if wm.streaming:
        wm.stream_data_callback(data.data)

if __name__ == "__main__":
    rospy.init_node("audio_player", anonymous=False)

    wm = WavToMouth()

    audio_player_service_param = rospy.get_param("services/audio_player/service", "/butia_speech/ap/audio_player")
    rospy.Service(audio_player_service_param, AudioPlayer, toTalk)

    audio_player_by_data_service_param = rospy.get_param("services/audio_player_by_data/service", "/butia_speech/ap/audio_player_by_data")
    rospy.Service(audio_player_by_data_service_param, AudioPlayerByData, toTalkByData)

    audio_player_stream_start_service_param = rospy.get_param("services/stream_start/service", "/butia_speech/ap/stream_start")
    rospy.Service(audio_player_stream_start_service_param, AudioStreamStart, audioStreamStart)

    audio_player_stream_stop_service_param = rospy.get_param("services/stream_stop/service", "/butia_speech/ap/stream_stop")
    rospy.Service(audio_player_stream_stop_service_param, Empty, stop_stream)

    audio_player_stream_data_topic_param = rospy.get_param("subscribers/stream_data/topic", "/butia_speech/ap/stream_data")
    rospy.Subscriber(audio_player_stream_data_topic_param, AudioData, stream_data_callback)

    print(colored("Audio Player is on!", "green"))
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if wm.streaming:
            wm.play_chunk()
        rate.sleep()
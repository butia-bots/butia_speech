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

last_stream_data_timestamp = None

wm = None

def toTalk(req):
    if wm.streaming:
        return AudioPlayerResponse(False)

    filepath = req.audio_path
    
    wm = WavToMouth(filepath=filepath)
    wm.read_audio()
    response = AudioPlayerResponse()
    response.success.data = True
    return response

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

    stream_timeout = rospy.get_param("stream_timeout", 10)

    print(colored("Audio Player is on!", "green"))
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if wm.streaming:
            if last_stream_data_timestamp is not None:
                now = rospy.Time.now()
                if now - last_stream_data_timestamp >= rospy.Duration(stream_timeout):
                    rospy.loginfo('STREAM TIMEOUT')
                    wm.request_stop_stream()
                    last_stream_data_timestamp = None
            wm.play_chunk()
        rate.sleep()
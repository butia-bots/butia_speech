#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from butia_speech.srv import SpeechToText, SpeechToTextResponse
from speech_recognition import Microphone, Recognizer, WaitTimeoutError, RequestError, UnknownValueError
import os
import numpy as np
from transformers import Wav2Vec2Processor, Wav2Vec2ForCTC, pipeline
from playsound import playsound

from RealtimeSTT import AudioToTextRecorder
import os

import warnings
warning = rospy.get_param("warnings", False)
if not warning:
    warnings.filterwarnings("ignore")

PACK_DIR = rospkg.RosPack().get_path("butia_speech")
RESOURCES_DIR = os.path.join(PACK_DIR, "resources")
AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
FILENAME = os.path.join(AUDIO_DIR, "speech_input.wav")
TALK_AUDIO = os.path.join(AUDIO_DIR, "beep.wav")

DEFAULT_LANGUAGE = 'en'

def handle_recognition(req):
    configs = rospt.getparam("/stt_configs/")
    rospy.loginfo("Initializing RealtimeSTT test... \n")

    if req.prompt != '':
        rospy.loginfo(f'Prompt to make easier the recognition: {req.prompt}')

    config.update({
        'language': req.language if req.language != '' else DEFAULT_LANGUAGE,
        'on_recording_start': lambda: rospy.loginfo("Starting Record..."),
        'on_vad_detect_start': lambda: playsound(TALK_AUDIO),
        'on_vad_detect_stop': lambda: rospy.loginfo("Finished Listening..."),
        'on_recording_stop': lambda: rospy.loginfo("Processing...")
        })

    try:
        with AudioToTextRecorder(**configs) as recorder:
            text = recorder.text()
        try:
            AudioToTextRecorder.shutdown()
        except:
            pass
    except Exception as e:
        print(e)
        text = ''
    return text

if __name__ == '__main__':
    rospy.init_node('speech_recognizer')
    
    recognizer_service_param = rospy.get_param("~services/speech_recognizer/service", "/butia_speech/sr/speech_recognizer")

    recognition_service = rospy.Service(recognizer_service_param, SpeechToText, handle_recognition)

    rospy.loginfo("Speech Recognizer is on!")
    rospy.spin()

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
    rospy.loginfo("Initializing RealtimeSTT test... \n")

    if req.prompt != '':
        rospy.loginfo(f'Prompt to make easier the recognition: {req.prompt}')

    recorder_config = {
        'compute_type': 'float32',
        'spinner': False,
        'model': 'base.en',
        'language': req.language if req.language != '' else DEFAULT_LANGUAGE,
        'silero_sensitivity': 0.6,
        'device': "cuda",
        'webrtc_sensitivity': 1,
        'post_speech_silence_duration': 0.2,
        'min_length_of_recording': 0.5,
        'min_gap_between_recordings': 0,
        'enable_realtime_transcription': False,
        'silero_deactivity_detection': True,
        'on_recording_start': print("Starting Record..."),
        'on_vad_detect_start': lambda: playsound(TALK_AUDIO),
        'on_vad_detect_stop': lambda: print("Finished Listening..."),
        'on_recording_stop': lambda: print("Processing..."),
    }

    try:
        with AudioToTextRecorder(**recorder_config) as recorder:
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
    recognizer = Recognizer()
    rospy.init_node('speech_recognizer')
    
    recognizer_service_param = rospy.get_param("~services/speech_recognizer/service", "/butia_speech/sr/speech_recognizer")
    noise_adjust_duration = rospy.get_param("~noise_adjust_duration", 1)
    whisper_model = rospy.get_param("~whisper_model", "small")
    phrase_time_limit = rospy.get_param("~phrase_time_limit", 3)
    timeout = rospy.get_param("~timeout", 3)
    sample_rate = rospy.get_param("~sample_rate", 16000)

    recognition_service = rospy.Service(recognizer_service_param, SpeechToText, handle_recognition)

    rospy.loginfo("Speech Recognizer is on!")
    rospy.spin()

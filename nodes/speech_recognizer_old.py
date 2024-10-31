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

from termcolor import colored
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
    with Microphone(sample_rate=sample_rate) as source:
        recognizer.adjust_for_ambient_noise(source, duration=noise_adjust_duration)

    playsound(TALK_AUDIO, block=True)

    with Microphone(sample_rate=sample_rate) as source:
        try:
            audio = recognizer.listen(source, phrase_time_limit=phrase_time_limit, timeout=timeout)
            
            prompt = req.prompt
            lang = req.lang

            if lang == '':
                lang = DEFAULT_LANGUAGE
            
            model = whisper_model
            if lang == 'en':
                if not model.endswith('.en'):
                    model = model + '.en'
            else:
                if model.endswith('.en'):
                    model = model[:-3]
            
            rospy.loginfo(f'Prompt to make easier the recognition: {prompt}')
            text = recognizer.recognize_whisper(audio, model, language=lang, initial_prompt=prompt, 
                                                load_options={'download_root': RESOURCES_DIR, 'in_memory': True}).lower()

        except WaitTimeoutError:
            text = ''
    return SpeechToTextResponse(
        text=text
    )

if __name__ == '__main__':
    recognizer = Recognizer()
    rospy.init_node('speech_recognizer')
    
    recognizer_service_param = rospy.get_param("~services/speech_recognizer/service", "/butia_speech/sr/speech_recognizer")
    noise_adjust_duration = rospy.get_param("~noise_adjust_duration", 1)
    whisper_model = rospy.get_param("~whisper_model", "small")
    phrase_time_limit = rospy.get_param("~phrase_time_limit", 8)
    timeout = rospy.get_param("~timeout", 5)
    sample_rate = rospy.get_param("~sample_rate", 16000)

    recognition_service = rospy.Service(recognizer_service_param, SpeechToText, handle_recognition)

    rospy.loginfo("Speech Recognizer is on!")
    rospy.spin()

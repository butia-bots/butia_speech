#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from butia_speech.srv import SpeechToText, SpeechToTextResponse
#from speech_recognition import Microphone, Recognizer, WaitTimeoutError, RequestError, UnknownValueError
import os
import numpy as np
from transformers import Wav2Vec2Processor, Wav2Vec2ForCTC, pipeline
from playsound import playsound

from RealtimeSTT import AudioToTextRecorder
from colorama import Fore, Back, Style
import colorama
import os

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
    '''with Microphone(sample_rate=sample_rate) as source:
        recognizer.adjust_for_ambient_noise(source, duration=noise_adjust_duration)'''
    
    print("Initializing RealtimeSTT test...")

    colorama.init()

    full_sentences = []
    global displayed_text
    displayed_text = ""
    
    def clear_console():
        os.system('clear' if os.name == 'posix' else 'cls')
        
    def text_detected(text):
        global displayed_text
        sentences_with_style = [
            f"{Fore.YELLOW + sentence + Style.RESET_ALL if i % 2 == 0 else Fore.CYAN + sentence + Style.RESET_ALL} "
            for i, sentence in enumerate(full_sentences)
        ]
        new_text = "".join(sentences_with_style).strip() + " " + text if len(sentences_with_style) > 0 else text

        if new_text != displayed_text:
            displayed_text = new_text
            clear_console()
            print(f"Language: {recorder.detected_language} (realtime: {recorder.detected_realtime_language})")
            print(displayed_text, end="", flush=True)
            #return displayed_text
    
    def process_text(text):
        full_sentences.append(text)
        text_detected("")
    
    
    prompt = req.prompt
    rospy.loginfo(f"prompt: {prompt}")

    recorder_config = {
        'spinner': False,
        'model': 'base.en',
        'silero_sensitivity': 0.7,
        'webrtc_sensitivity': 1,
        'post_speech_silence_duration': 0.4,
        'min_length_of_recording': 1,
        'min_gap_between_recordings': 0,
        'enable_realtime_transcription': False,
        'realtime_processing_pause': 0.1,
        'realtime_model_type': 'tiny.en',
        'on_realtime_transcription_update': text_detected, 
        'silero_deactivity_detection': True,
    }

    with AudioToTextRecorder(**recorder_config) as recorder:
        try:
            start_time = rospy.Time.now()
            rospy.logwarn("aqui vem")
            playsound(TALK_AUDIO, block=False)
            recorder.text(process_text)

            rospy.logwarn(f"Tempo do recorder: {(rospy.Time.now() - start_time).to_sec()}")            
            lang = req.lang
            rospy.logwarn(f'Prompt to make easier the recognition: {prompt}')
            
            text = displayed_text
        except WaitTimeoutError:
            text = ''
    rospy.loginfo(f"Audio reconhecido {text}")
    '''
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
            rospy.logwarn("aqui vem")
            rospy.loginfo(f'Prompt to make easier the recognition: {prompt}')
            text = recognizer.recognize_whisper(audio, model, language=lang, initial_prompt=prompt, 
                                                load_options={'download_root': RESOURCES_DIR, 'in_memory': True}).lower()

        except WaitTimeoutError:
            text = ''
    '''
    return SpeechToTextResponse(
        text=text
    )

if __name__ == '__main__':
    #recognizer = Recognizer()
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

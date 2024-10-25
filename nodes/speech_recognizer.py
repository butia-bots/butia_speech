#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import rospkg
from butia_speech.srv import SpeechToText, SpeechToTextResponse
import os
import numpy as np
from playsound import playsound

from RealtimeSTT import AudioToTextRecorder
import os
import time

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
    default_config = {
        "compute_type": "float32",
        "spinner": False,
        "model": "small.en",
        "silero_sensitivity": 0.5,
        "device": "cpu",
        "webrtc_sensitivity": 1,
        "post_speech_silence_duration": 0.4,
        "min_length_of_recording": 0.5,
        "min_gap_between_recordings": 0,
        "enable_realtime_transcription": False,
        "silero_deactivity_detection": True,
    }

    # Fetch the STT configurations from ROS parameters
    configs = rospy.get_param("~stt_configs/", default_config)

    # If a prompt is provided, update the configurations
    if req.prompt != '':
        rospy.loginfo(f'Prompt to make easier the recognition: {req.prompt}')
        configs.update({'initial_prompt': req.prompt})

    # Update the configurations with additional parameters
    configs.update({
        'language': req.lang if req.lang != '' else DEFAULT_LANGUAGE,  # Set the language for recognition
        'on_recording_start': lambda: rospy.loginfo("Starting Record..."),  # Log message when recording starts
        'on_vad_detect_start': lambda: playsound(TALK_AUDIO),  # Play beep sound when voice activity is detected
        'on_vad_detect_stop': lambda: rospy.loginfo("Finished Listening..."),  # Log message when voice activity stops
        'on_recording_stop': lambda: rospy.loginfo("Processing...")  # Log message when recording stops
    })

    try:
        start_time = time.time()  # Start the timer
        # Initialize the audio-to-text recorder with the configurations
        with AudioToTextRecorder(**configs) as recorder:
            # Get the recognized text
            text = recorder.text()
            elapsed_time = time.time() - start_time  # Calculate elapsed time
            if elapsed_time > 7:
                raise TimeoutError("Recording exceeded 7 seconds")
        try:
            # Shutdown the recorder
            AudioToTextRecorder.shutdown()
        except:
            pass
    except Exception as e:
        # Print any exceptions that occur
        print(e)
        text = ''
    return SpeechToTextResponse(
        text=text
    )

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('speech_recognizer')
    
    # Fetch the recognizer service parameter
    recognizer_service_param = rospy.get_param("~services/speech_recognizer/service", "/butia_speech/sr/speech_recognizer")

    # Provide the speech recognition service
    recognition_service = rospy.Service(recognizer_service_param, SpeechToText, handle_recognition)

    rospy.loginfo("Speech Recognizer is on!")
    # Keep the node running
    rospy.spin()
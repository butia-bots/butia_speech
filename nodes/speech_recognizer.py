#!/usr/bin/env python3

import rospy
from butia_speech.srv import SpeechToText, SpeechToTextResponse
from speech_recognition import Microphone, Recognizer, WaitTimeoutError
import os
import numpy as np
import deepspeech

deepspeech_model_dir = os.path.join(os.path.dirname(__file__), '../include/deepspeech/')


def handle_recognition(req):
    with Microphone(sample_rate=deepspeech_model.sampleRate()) as source:
        recognizer.adjust_for_ambient_noise(source, duration=0.5)
        rospy.loginfo('Listening ...')
        try:
            audio = recognizer.listen(source, phrase_time_limit=10, timeout=3)
            fs = audio.sample_rate
            audio = np.frombuffer(audio.frame_data, np.int16)
            text = deepspeech_model.stt(audio)
        except WaitTimeoutError:
            text = ''
    return SpeechToTextResponse(
        text=text
    )

if __name__ == '__main__':
    rospy.init_node('speech_recognizer')
    recognizer = Recognizer()
    deepspeech_model = deepspeech.Model(deepspeech_model_dir + 'deepspeech-0.9.3-models.pbmm')
    deepspeech_model.enableExternalScorer(deepspeech_model_dir + 'deepspeech-0.9.3-models.scorer')
    recognition_service = rospy.Service('/butia_speech/asr/transcribe', SpeechToText, handle_recognition)
    rospy.spin()

#!/usr/bin/env python3

import rospy
from butia_speech.srv import SpeechToText, SpeechToTextResponse
from butia_speech.speech_to_text import ButiaSpeechToText
from speech_recognition import Microphone

def handle_recognition(req):
    with Microphone() as source:
        audio = recognizer.listen(source, phrase_time_limit=8)
    return SpeechToTextResponse(
        text=recognizer.recognize_deepspeech(audio)
    )

if __name__ == '__main__':
    rospy.init_node('speech_recognizer')
    recognizer = ButiaSpeechToText()
    recognition_service = rospy.Service('/butia_speech/transcribe', SpeechToText, handle_recognition)
    rospy.spin()

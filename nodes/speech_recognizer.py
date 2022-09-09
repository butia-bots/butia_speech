#!/usr/bin/env python3

import rospy
import rospkg
from butia_speech.srv import SpeechToText, SpeechToTextResponse
from speech_recognition import Microphone, Recognizer, WaitTimeoutError, RequestError, UnknownValueError
import os
import numpy as np
from transformers import Wav2Vec2Processor, Wav2Vec2ForCTC, pipeline
from playsound import playsound

AUDIO_DIR = os.path.join(rospkg.RosPack().get_path("butia_speech"), "audios/")
FILENAME = os.path.join(AUDIO_DIR, "speech_input.wav")
TALK_AUDIO = os.path.join(AUDIO_DIR, "beep.wav")

def handle_recognition(req):
    with Microphone(sample_rate=16000) as source:
        recognizer.adjust_for_ambient_noise(source, duration=1)

    playsound(TALK_AUDIO, block=True)

    with Microphone(sample_rate=16000) as source:
        try:
            audio = recognizer.listen(source, phrase_time_limit=20, timeout=5)
            text = ''

            if online_preference:
                try:
                    text = recognizer.recognize_google(audio)
                except (UnknownValueError, RequestError):
                    text = ''
        
            if text == '':
                with open(FILENAME, 'wb') as f:
                    f.write(audio.get_wav_data())
                text = asr_pipeline(FILENAME)['text'].lower()
        except WaitTimeoutError:
            text = ''
    return SpeechToTextResponse(
        text=text
    )

if __name__ == '__main__':
    #processor = Wav2Vec2Processor.from_pretrained("facebook/wav2vec2-large-960h-lv60")
    #model = Wav2Vec2ForCTC.from_pretrained("facebook/wav2vec2-large-960h-lv60")
    asr_pipeline = pipeline(task="automatic-speech-recognition", model="facebook/wav2vec2-large-960h-lv60")
    recognizer = Recognizer()
    rospy.init_node('speech_recognizer')
    
    recognizer_service_param = rospy.get_param("~services/speech_recognizer/service", "/butia_speech/sr/speech_recognizer")
    online_preference = rospy.get_param("~online_preference", True)

    recognition_service = rospy.Service(recognizer_service_param, SpeechToText, handle_recognition)
    rospy.spin()

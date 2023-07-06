#!/usr/bin/env python3

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
AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
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
                #with open(FILENAME, 'wb') as f:
                #    f.write(audio.get_wav_data())

                names = rospy.get_param('/people_names', [])
                drinks = rospy.get_param('/drink_names', [])

                has_param = False
                prompt = 'Just the words in the following list must be detected, that are: '
                if len(names) > 0:
                    prompt += ', '.join(names) + ', '
                    has_param = True
                if len(drinks) > 0:
                    prompt += ', '.join(drinks) + ', '
                    has_param = True
                prompt += 'yes, no.'

                prompt = prompt if has_param else None
                
                rospy.loginfo(f'Prompt to make easier the recognition: {prompt}')
                text = recognizer.recognize_whisper(audio, 'small', language='en', initial_prompt=prompt).lower()
                #text = model.transcribe(FILENAME, no_speech_threshold=float('inf'), compression_ratio_threshold=float('inf'), initial_prompt=prompt)['text'].lower()

        except WaitTimeoutError:
            text = ''
    return SpeechToTextResponse(
        text=text
    )

if __name__ == '__main__':
    #processor = Wav2Vec2Processor.from_pretrained("facebook/wav2vec2-large-960h-lv60")
    #model = Wav2Vec2ForCTC.from_pretrained("facebook/wav2vec2-large-960h-lv60")
    # print("**********************************************************")
    # try:
    #     asr_pipeline = pipeline(task="automatic-speech-recognition", model=os.path.join(PACK_DIR, "include/model/facebook"))
    #     print("************Local model loaded**************")
    # except:
    #     print("************Local model failed, downloading from internet**************")
    #     asr_pipeline = pipeline(task="automatic-speech-recognition", model="facebook/wav2vec2-large-960h-lv60")
    #     asr_pipeline.save_pretrained(os.path.join(PACK_DIR, "include/model/facebook"))

    #model = whisper.load_model("base.en")

    recognizer = Recognizer()
    rospy.init_node('speech_recognizer')
    
    recognizer_service_param = rospy.get_param("~services/speech_recognizer/service", "/butia_speech/sr/speech_recognizer")
    online_preference = rospy.get_param("~online_preference", False)

    recognition_service = rospy.Service(recognizer_service_param, SpeechToText, handle_recognition)

    print(colored("Speech Recognizer is on!", "green"))
    rospy.spin()

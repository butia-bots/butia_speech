#!/usr/bin/env python3
# coding: utf-8
from butia_speech.srv import AudioPlayer, SynthesizeSpeech, SynthesizeSpeechResponse
from butia_speech.msg import SynthesizeSpeechMessage
from std_msgs.msg import Bool

from scipy.io import wavfile
import os
import rospy
import numpy as np
import rospkg

import numpy as np
import riva.client

from termcolor import colored
import warnings
warning = rospy.get_param("warnings", False)
if not warning:
    warnings.filterwarnings("ignore")
PACK_DIR = rospkg.RosPack().get_path("butia_speech")
AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
FILENAME = str(AUDIO_DIR) + "talk.wav"
#MODEL_DIR = os.path.join(PACK_DIR, "include/model/total_count/")
#MODEL_NAME = "model.pkl"
#MODEL_PATH = os.path.join(MODEL_DIR, MODEL_NAME)

sample_rate_hz = 44100
riva_req = { 
        "language_code"  : "en-US",
        "encoding"       : riva.client.AudioEncoding.LINEAR_PCM ,   # LINEAR_PCM and OGGOPUS encodings are supported
        "sample_rate_hz" : sample_rate_hz,                          # Generate 44.1KHz audio
        "voice_name"     : "English-US.Male-1",                 # The name of the voice to generate
}

def synthesize_speech(req):
    speech = req.text
    lang = "en" # lang = req.lang
    
    resp = riva_tts.synthesize(
        custom_dictionary=riva_req, 
        text=speech,
        voice_name="English-US-RadTTS.Male-1", 
        sample_rate_hz=sample_rate_hz, 
        encoding=riva.client.AudioEncoding.LINEAR_PCM,
        )
    audio_samples = np.frombuffer(resp.audio, dtype=np.int16)
    try:
        wavfile.write(FILENAME, sample_rate_hz, audio_samples)
        print("success")
    except:
        print("error")
    
    audio_player_service_param = rospy.get_param("services/audio_player/service", "/butia_speech/ap/audio_player")
    rospy.wait_for_service(audio_player_service_param, timeout=rospy.Duration(10))
    try:
        audio_player = rospy.ServiceProxy(audio_player_service_param, AudioPlayer)
        audio_player(FILENAME)
<<<<<<< HEAD

        response = SynthesizeSpeechResponse()
        response.success = True
        return response
=======
        msg = SynthesizeSpeechResponse()
        msg.success.data = True
        return msg
>>>>>>> bf99cb3 (addind the attribute data to the response)
    except rospy.ServiceException as exc:
        response = SynthesizeSpeechResponse
        response.success = False
        print("Service call failed: %s" % exc)
<<<<<<< HEAD
        return response
=======
        msg = SynthesizeSpeechResponse()
        msg.success.data = False
        return msg
>>>>>>> bf99cb3 (addind the attribute data to the response)


if __name__ == '__main__':
    auth = riva.client.Auth(uri='jetson:50051')
    riva_tts = riva.client.SpeechSynthesisService(auth)

    rospy.init_node('speech_synthesizer', anonymous=False)

    say_something_subscriber_param = rospy.get_param("subscribers/speech_synthesizer/topic", "/butia_speech/ss/say_something")
    rospy.Subscriber(say_something_subscriber_param, SynthesizeSpeechMessage, callback=synthesize_speech)

    synthesizer_service_param = rospy.get_param("services/speech_synthesizer/service", "/butia_speech/ss/say_something")
    rospy.Service(synthesizer_service_param, SynthesizeSpeech, synthesize_speech)
    
    print(colored("Speech Synthesizer is on!", "green"))
    rospy.spin()

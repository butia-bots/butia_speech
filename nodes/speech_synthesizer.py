#!/usr/bin/env python3
# coding: utf-8
from butia_speech.srv import AudioPlayer, SynthesizeSpeech, SynthesizeSpeechResponse
from butia_speech.msg import SynthesizeSpeechMessage
from std_msgs.msg import Bool

from espnet2.bin.tts_inference import Text2Speech
from espnet2.utils.types import str_or_none
from scipy.io import wavfile
import os
import torch
import rospy
import numpy as np
import rospkg

from termcolor import colored
import warnings
warning = rospy.get_param("warnings", False)
if not warning:
    warnings.filterwarnings("ignore")

AUDIO_DIR = os.path.join(rospkg.RosPack().get_path("butia_speech"), "audios/")
FILENAME = str(AUDIO_DIR) + "talk.wav"

def speak_in_english(speech):
    with torch.no_grad():
        wav = text2speech_english(speech)["wav"]
        wavfile.write(FILENAME, text2speech_english.fs, (wav.view(-1).cpu().numpy()*32768).astype(np.int16))

def speak_in_portuguese(speech):
    with torch.no_grad():
        wav = text2speech_portuguese(speech)["wav"]
        wavfile.write(FILENAME, text2speech_portuguese.fs, (wav.view(-1).cpu().numpy()*32768).astype(np.int16))

def synthesize_speech(req):
    speech = req.text
    try:
        lang = req.lang
    except:
        lang = "en"

    if lang == "pt":
        speak_in_portuguese(speech)
    else:
        speak_in_english(speech)
    
    audio_player_service_param = rospy.get_param("services/audio_player/service", "/butia_speech/ap/audio_player")
    rospy.wait_for_service(audio_player_service_param, timeout=rospy.Duration(10))
    try:
        audio_player = rospy.ServiceProxy(audio_player_service_param, AudioPlayer)
        audio_player(FILENAME)
    
        return SynthesizeSpeechResponse(Bool(True))
    except rospy.ServiceException as exc:
        print("Service call failed: %s" % exc)
        return SynthesizeSpeechResponse(Bool(False))


if __name__ == '__main__':
    print("Get model in english...")
    text2speech_english = Text2Speech.from_pretrained(model_tag=str_or_none("kan-bayashi/ljspeech_vits"),
                                                      vocoder_tag=str_or_none("none"),
                                                      device="cpu",
                                                      threshold=0.5,
                                                      minlenratio=0.0,
                                                      maxlenratio=10.0,
                                                      use_att_constraint=False,
                                                      backward_window=1,
                                                      forward_window=3,
                                                      speed_control_alpha=1.15,
                                                      noise_scale=0.333,
                                                      noise_scale_dur=0.333,
                                                    )
    print("Done.")

    print("Get model in portuguese...")
    text2speech_portuguese = Text2Speech.from_pretrained(model_tag="espnet/pt_commonvoice_blstm",
                                                        #  vocoder_tag="none",
                                                        #  device="cpu",
                                                        #  threshold=0.5,
                                                        #  minlenratio=0.0,
                                                        #  maxlenratio=10.0,
                                                        #  use_att_constraint=False,
                                                        #  backward_window=1,
                                                        #  forward_window=3,
                                                        #  speed_control_alpha=1.15,
                                                        #  noise_scale=0.333,
                                                        #  noise_scale_dur=0.333
                                                        )
    print("Done.")

    rospy.init_node('speech_synthesizer', anonymous=False)

    say_something_subscriber_param = rospy.get_param("subscribers/speech_synthesizer/topic", "/butia_speech/ss/say_something")
    rospy.Subscriber(say_something_subscriber_param, SynthesizeSpeechMessage, callback=synthesize_speech)

    synthesizer_service_param = rospy.get_param("services/speech_synthesizer/service", "/butia_speech/ss/say_something")
    rospy.Service(synthesizer_service_param, SynthesizeSpeech, synthesize_speech)
    
    print(colored("Speech Synthesizer is on!", "green"))
    rospy.spin()

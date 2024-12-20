#!/usr/bin/env python3
# coding: utf-8
from butia_speech.srv import AudioPlayer, AudioPlayerByData, AudioPlayerByDataRequest, SynthesizeSpeech, SynthesizeSpeechResponse
from butia_speech.msg import SynthesizeSpeechMessage
from audio_common_msgs.msg import AudioData, AudioInfo
from std_msgs.msg import Bool

from espnet2.bin.tts_inference import Text2Speech
from espnet2.utils.types import str_or_none
from scipy.io import wavfile
import os
import torch
import rospy
import numpy as np
import rospkg
import pickle

from termcolor import colored
import warnings
warning = rospy.get_param("warnings", False)
if not warning:
    warnings.filterwarnings("ignore")
PACK_DIR = rospkg.RosPack().get_path("butia_speech")
AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
FILENAME = str(AUDIO_DIR) + "talk.wav"
MODEL_DIR = os.path.join(PACK_DIR, "include/model/total_count/")
MODEL_NAME = "model.pkl"
MODEL_PATH = os.path.join(MODEL_DIR, MODEL_NAME)


def synthesize_speech(req):
    speech = req.text
    lang = "en"  # lang = req.lang

    with torch.no_grad():
        wav = text2speech(speech)["wav"]
        wav_data = (wav.view(-1).cpu().numpy() * 32768).astype(np.int16)
        wavfile.write(FILENAME, text2speech.fs, wav_data)

    audio_data = AudioData()
    audio_data.data = wav_data.tobytes()
    audio_info = AudioInfo()
    audio_info.sample_rate = text2speech.fs
    audio_info.channels = 1
    audio_info.sample_format = '16'  # Assuming 16-bit PCM

    # Fetch the audio player by data service parameter
    audio_player_by_data_service_param = rospy.get_param("services/audio_player_by_data/service", "/butia_speech/ap/audio_player_by_data")
    # Wait for the audio player by data service to be available
    rospy.wait_for_service(audio_player_by_data_service_param, timeout=rospy.Duration(10))
    try:
        audio_player_by_data = rospy.ServiceProxy(audio_player_by_data_service_param, AudioPlayerByData)
        
        request = AudioPlayerByDataRequest()
        request.data = audio_data
        request.audio_info = audio_info
        audio_player_by_data(request)
        
        response = True
        return response
    except rospy.ServiceException as exc:
        response = False
        print("Service call failed: %s" % exc)
        return response


if __name__ == '__main__':

    tag = rospy.get_param("butia_speech_synthesizer/tag", "kan-bayashi/ljspeech_vits")
    vocoder_tag = rospy.get_param("butia_speech_synthesizer/vocoder_tag", "none")
    os.makedirs(MODEL_DIR, exist_ok=True)
    try:
        with open(os.path.join(PACK_DIR, MODEL_PATH),'rb') as f:
            text2speech = pickle.load(f)
        print("-------------Local model loaded-------------")
    except Exception as e:
        print(f"Failed to load local model error: {e}")
        text2speech = Text2Speech.from_pretrained(model_tag=str_or_none(tag),
                                              vocoder_tag=str_or_none(vocoder_tag),
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
        print(text2speech)
        print("-----------Download from internet----------------")
        with open(os.path.join(PACK_DIR, MODEL_PATH), 'wb') as f:
            pickle.dump(text2speech, f)
    
    rospy.init_node('speech_synthesizer', anonymous=False)

    say_something_subscriber_param = rospy.get_param("subscribers/speech_synthesizer/topic", "/butia_speech/ss/say_something")
    rospy.Subscriber(say_something_subscriber_param, SynthesizeSpeechMessage, callback=synthesize_speech)

    synthesizer_service_param = rospy.get_param("services/speech_synthesizer/service", "/butia_speech/ss/say_something")
    rospy.Service(synthesizer_service_param, SynthesizeSpeech, synthesize_speech)
    
    print(colored("Speech Synthesizer is on!", "green"))
    rospy.spin()

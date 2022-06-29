#!/usr/bin/env python3
# coding: utf-8
from butia_speech.srv import AudioPlayer, SynthesizeSpeech, SynthesizeSpeechResponse
from std_msgs.msg import Bool, String
from espnet2.bin.tts_inference import Text2Speech
from espnet2.utils.types import str_or_none
from scipy.io import wavfile
import os
import torch
import rospy
import numpy as np
import rospkg

AUDIO_DIR = os.path.join(rospkg.RosPack().get_path("butia_speech"), "audios/")
FILENAME = str(AUDIO_DIR) + "talk.wav"

tag = 'kan-bayashi/ljspeech_vits'
vocoder_tag = "none"

def synthesize_speech(req):
    try:
        speech = req.data
    except:
        speech = req.text

    with torch.no_grad():
        wav = text2speech(speech)["wav"]
        wavfile.write(FILENAME, text2speech.fs, (wav.view(-1).cpu().numpy()*32768).astype(np.int16))
    
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
    text2speech = Text2Speech.from_pretrained(
        model_tag=str_or_none(tag),
        vocoder_tag=str_or_none(vocoder_tag),
        device="cpu",
        # Only for Tacotron 2 & Transformer
        threshold=0.5,
        # Only for Tacotron 2
        minlenratio=0.0,
        maxlenratio=10.0,
        use_att_constraint=False,
        backward_window=1,
        forward_window=3,
        # Only for FastSpeech & FastSpeech2 & VITS
        speed_control_alpha=1.0,
        # Only for VITS
        noise_scale=0.333,
        noise_scale_dur=0.333,
    )
    rospy.init_node('speech_synthesizer', anonymous=False)

    say_something_subscriber_param = rospy.get_param("subscribers/butia_speech/topic", "/butia_speech/bs/say_something")
    rospy.Subscriber(say_something_subscriber_param, String, callback=synthesize_speech)

    synthesizer_service_param = rospy.get_param("services/speech_synthesizer/service", "/butia_speech/ss/speech_synthesizer")
    rospy.Service(synthesizer_service_param, SynthesizeSpeech, synthesize_speech)
    
    rospy.spin()

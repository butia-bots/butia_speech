#!/usr/bin/env python3
# coding: utf-8
from butia_speech.srv import *
from std_msgs.msg import Bool
from espnet2.bin.tts_inference import Text2Speech
from espnet2.utils.types import str_or_none
from scipy.io import wavfile
from pygame import mixer
import torch
import time
import rospy
import numpy as np

tag = 'kan-bayashi/ljspeech_vits'
vocoder_tag = "none"

def synthesize_speech(req):
    #tts = gTTS(req.speech, lang=req.lang)
    #tts.save('speech.mp3')
    #mixer.init()
    #mixer.music.load('speech.mp3')
    #mixer.music.play()
    #return SynthesizeSpeechResponse(Bool(True))
    # synthesis
    with torch.no_grad():
        start = time.time()
        wav = text2speech(req.speech)["wav"]
        wavfile.write('fala.wav', text2speech.fs, (wav.view(-1).cpu().numpy()*32768).astype(np.int16))
    mixer.init()
    mixer.music.load('fala.wav')
    mixer.music.play()
    return SynthesizeSpeechResponse(Bool(True))

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
    rospy.init_node('speech_synthesizer')
    rospy.Service('butia/synthesize_speech', SynthesizeSpeech, synthesize_speech)
    rospy.spin()

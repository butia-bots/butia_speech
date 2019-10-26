#!/usr/bin/env python
# coding: utf-8
from butia_speech.srv import *
from std_msgs.msg import Bool
from gtts import gTTS, lang
from pygame import mixer
import rospy

def synthesize_speech(req):
    #tts = gTTS(req.speech, lang=req.lang)
    #tts.save('speech.mp3')
    #mixer.init()
    #mixer.music.load('speech.mp3')
    #mixer.music.play()
    #return SynthesizeSpeechResponse(Bool(True))
    mixer.init()
    mixer.music.load('fala.mpeg')
    mixer.music.play()
    return SynthesizeSpeechResponse(Bool(True))

if __name__ == '__main__':
    rospy.init_node('speech_synthesizer')
    rospy.Service('butia/synthesize_speech', SynthesizeSpeech, synthesize_speech)
    rospy.spin()

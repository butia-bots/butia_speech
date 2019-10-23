#!/usr/bin/env python
import rospy
import speech_recognition as sr
from std_msgs.msg import String
from butia_speech.srv import SynthesizeSpeech
from speech_synthesizer import synthesize_speech

r = sr.Recognizer()

if __name__ == "__main__":
    rospy.init_node("speech_to_text", anonymous=True)
    stt_publisher = rospy.Publisher("/butia_speech/stt/transcribe", String, queue_size=1)
    while not rospy.is_shutdown():
        with sr.Microphone() as source:
            audio = r.listen(source)
            try:
                output = r.recognize_google(audio)
                transcribe = String()
                transcribe.data = output
                stt_publisher.publish(transcribe)
            except sr.UnknownValueError:
                req = SynthesizeSpeech()
                req.lang = "en-us"
                req.speech = "I do not understand what you say. Can you repeat?"
                synthesize_speech(req)

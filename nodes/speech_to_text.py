#!/usr/bin/env python
import rospy
import speech_recognition as sr
from std_msgs.msg import String
from butia_speech.srv import SynthesizeSpeech

r = sr.Recognizer()

if __name__ == "__main__":
    rospy.init_node("speech_to_text", anonymous=True)
    stt_publisher = rospy.Publisher("/butia_speech/stt/transcribe", String, queue_size=1)

    rospy.wait_for_service("/butia/synthesize_speech")
    try:
        synt_voice = rospy.ServiceProxy("/butia/synthesize_speech", SynthesizeSpeech)
    except rospy.ServiceException, e:
        print "Service call failed %s"%e

    while not rospy.is_shutdown():
        with sr.Microphone() as source:
            audio = r.listen(source)
            try:
                output = r.recognize_google(audio)
                transcribe = String()
                transcribe.data = output
                stt_publisher.publish(transcribe)
            except sr.UnknownValueError:
                lang = "en-us"
                speech = "I do not understand what you say. Can you repeat?"
                synt_voice(speech, lang)

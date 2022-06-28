#!/usr/bin/env python3

import rospy

from std_msgs.msg import String

from butia_speech.wav_to_mouth import WavToMouth
from butia_speech.srv import SynthesizeSpeechRequest, SynthesizeSpeech

def toTalk(msg):
    speech = msg.data
    
    request = SynthesizeSpeechRequest()
    request.speech = speech
    request.lang = "en" # receive too

    butia_text_to_speech = rospy.get_param("services/text_to_speech", "butia/synthesize_speech")

    rospy.wait_for_service(butia_text_to_speech, timeout=rospy.Duration(10))
    try:
        synthesize_speech = rospy.ServiceProxy(butia_text_to_speech, SynthesizeSpeech)
        synthesize_speech(request)

        w2m = WavToMouth()
        w2m.read_audio()

    except rospy.ServiceException as exc:
        print("Service call failed: %s" % exc)

if __name__ == "__main__":
    rospy.init_node("audio_player", anonymous=False)

    rospy.Subscriber("receivePhrase", String, callback=toTalk)
    rospy.spin()
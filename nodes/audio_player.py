#!/usr/bin/env python3

import rospy

from std_msgs.msg import String

from butia_speech.wav_to_mouth import WavToMouth
from butia_speech.srv import SynthesizeSpeechRequest, SynthesizeSpeech

def toTalk(msg):
    speech = msg.data
    
    request = SynthesizeSpeechRequest()
    request.speech = speech
    request.lang = "en"

    synthesizer_service_param = rospy.get_param("services/speech_synthesizer/service", "butia_speech/ss/speech_synthesizer")

    rospy.wait_for_service(synthesizer_service_param, timeout=rospy.Duration(10))
    try:
        synthesize_speech = rospy.ServiceProxy(synthesizer_service_param, SynthesizeSpeech)
        synthesize_speech(request)

        w2m = WavToMouth()
        w2m.read_audio()

    except rospy.ServiceException as exc:
        print("Service call failed: %s" % exc)

if __name__ == "__main__":
    rospy.init_node("audio_player", anonymous=False)

    rospy.Subscriber("receivePhrase", String, callback=toTalk)
    rospy.spin()
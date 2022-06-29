#!/usr/bin/env python3

import rospy

from std_msgs.msg import String

from butia_speech.wav_to_mouth import WavToMouth
from butia_speech.srv import SynthesizeSpeech, SynthesizeSpeechRequest, AudioPlayer

def toTalk(req):
    try:
        speech = req.data
    except:
        speech = req.speech
    
    rospy.loginfo(f'speech: {speech}')
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

    say_something_subscriber_param = rospy.get_param("subscribers/butia_speech/topic", "/butia_speech/bs/say_something")
    rospy.Subscriber(say_something_subscriber_param, String, callback=toTalk)

    audio_player_service_param = rospy.get_param("services/audio_player/service", "/butia_speech/ap/audio_player")
    rospy.Service(audio_player_service_param, AudioPlayer, toTalk)

    rospy.spin()
#!/usr/bin/env python3
# coding: utf-8
from butia_speech.srv import AudioPlayer,AudioPlayerByData, AudioPlayerByDataRequest, SynthesizeSpeech, SynthesizeSpeechResponse
from butia_speech.msg import SynthesizeSpeechMessage
from std_msgs.msg import Bool
from audio_common_msgs.msg import AudioData, AudioInfo

from scipy.io import wavfile
import os
import rospy
import numpy as np
import rospkg

import numpy as np
import riva.client

from termcolor import colored
import warnings


# Fetch the warning parameter from ROS parameters
warning = rospy.get_param("warnings", False)
if not warning:
    # Suppress warnings if the parameter is set to False
    warnings.filterwarnings("ignore")

# Get the path of the 'butia_speech' package
PACK_DIR = rospkg.RosPack().get_path("butia_speech")
# Construct the path to the audio directory
AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
# Set the filename for the generated audio file
FILENAME = str(AUDIO_DIR) + "talk.wav"

def synthesize_speech(req):
    config_defaults = {
        "language_code": "en-US",
        "sample_rate_hz": 44100,
        "voice_name": "English-US.Male-1",
    }
    configs = rospy.get_param("tts_configs/", config_defaults)

    # Extract the text to be synthesized from the request
    speech = req.text
    
    # Call Riva TTS to synthesize the speech
    resp = riva_tts.synthesize(
        custom_dictionary=configs, 
        text=speech,
        voice_name=configs["voice_name"], 
        sample_rate_hz=configs["sample_rate_hz"],
        language_code=configs["language_code"],
        encoding=riva.client.AudioEncoding.LINEAR_PCM,
    )
    # Convert the response audio to a NumPy array
    audio_samples = np.frombuffer(resp.audio, dtype=np.int16)
    
    audio_data = AudioData()
    audio_data.data = audio_samples.tobytes()
    audio_info = AudioInfo()
    audio_info.sample_rate = configs["sample_rate_hz"]
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
        
        response = SynthesizeSpeechResponse()
        response.success = True
        return response
    except rospy.ServiceException as exc:
        response = SynthesizeSpeechResponse()
        response.success = False
        print("Service call failed: %s" % exc)
        return response


if __name__ == '__main__':

    # Initialize the ROS node
    rospy.init_node('speech_synthesizer', anonymous=False)

    riva_url = rospy.get_param("~riva/url", "localhost:50051")
    # Authenticate with the Riva server
    auth = riva.client.Auth(uri=riva_url)
    # Initialize the Riva TTS service
    riva_tts = riva.client.SpeechSynthesisService(auth)

    # Fetch the subscriber topic parameter
    say_something_subscriber_param = rospy.get_param("subscribers/speech_synthesizer/topic", "/butia_speech/ss/say_something")
    # Subscribe to the topic to receive text messages
    rospy.Subscriber(say_something_subscriber_param, SynthesizeSpeechMessage, callback=synthesize_speech)

    # Fetch the service parameter
    synthesizer_service_param = rospy.get_param("services/speech_synthesizer/service", "/butia_speech/ss/say_something")
    # Provide the speech synthesis service
    rospy.Service(synthesizer_service_param, SynthesizeSpeech, synthesize_speech)
    
    print(colored("Speech Synthesizer is on!", "green"))
    # Keep the node running
    rospy.spin()
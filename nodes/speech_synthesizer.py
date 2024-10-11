#!/usr/bin/env python3
# coding: utf-8
from butia_speech.srv import AudioPlayer, SynthesizeSpeech, SynthesizeSpeechResponse
from butia_speech.msg import SynthesizeSpeechMessage
from std_msgs.msg import Bool

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
    riva_url = rospy.get_param("~riva/url", "localhost:50051")
    configs = rospy.get_param("tts_configs/")
    
    # Authenticate with the Riva server
    auth = riva.client.Auth(uri=riva_url)
    # Initialize the Riva TTS service
    riva_tts = riva.client.SpeechSynthesisService(auth)
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
    try:
        # Write the audio samples to a WAV file
        wavfile.write(FILENAME, configs["sample_rate_hz"], audio_samples)
        print("success")  # Print success message if the file is written successfully
    except:
        print("error")  # Print error message if there is an issue writing the file
    
    # Fetch the audio player service parameter
    audio_player_service_param = rospy.get_param("services/audio_player/service", "/butia_speech/ap/audio_player")
    # Wait for the audio player service to be available
    rospy.wait_for_service(audio_player_service_param, timeout=rospy.Duration(10))
    try:
        audio_player = rospy.ServiceProxy(audio_player_service_param, AudioPlayer)
        audio_player(FILENAME)

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
#!/usr/bin/env python3

import rospy
import rospkg
from butia_speech.srv import AudioPlayer, SynthesizeSpeech, SynthesizeSpeechResponse, SynthesizeSpeechRequest
from butia_speech.msg import SynthesizeSpeechMessage
from std_msgs.msg import Bool

import os
from termcolor import colored
from scipy.io import wavfile
import torch
import numpy as np

from TTS.tts.configs.xtts_config import XttsConfig
from TTS.tts.models.xtts import Xtts
from TTS.api import TTS

HOME_DIR = os.path.expanduser("~")

PACK_DIR = rospkg.RosPack().get_path("butia_speech")
VOICES_DIR = os.path.join(PACK_DIR, "voices/")
MODELS_DIR = os.path.join(HOME_DIR, ".local/share/tts")

AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
FILENAME = str(AUDIO_DIR) + "talk.wav"

WAV_EXTENSION = ".wav"

class XTTSSpeechSynthesizerNode:
    def __init__(self, node_name="speech_synthesizer", anonymous=False):
        rospy.init_node(node_name, anonymous=anonymous)
        self.__read_params()

        self.sample_rate = 24000

        rospy.loginfo("Loading TTS model: " + self.tts_model_name)
        try:
            TTS(self.tts_model_name)
        except Exception as e:
            rospy.logerr("Error loading TTS model: " + str(e))
            rospy.logerr("Use some model from the list: " + str(TTS.list_models()))
            return

        self.tts_model_dir = os.path.join(MODELS_DIR, self.tts_model_name.replace("/", "--"))
        rospy.logdebug("TTS model directory: " + self.tts_model_dir)

        if not os.path.exists(self.tts_model_dir):
            rospy.logerr("TTS model directory not found: " + self.tts_model_dir)
            exit()
        
        self.__load_tts_model()
        self.__compute_voice_latents()

        self.__init_comm()

        print(colored("Speech Synthesizer is on!", "green"))
    
    def __load_tts_model(self):
        config = XttsConfig()
        config.load_json(os.path.join(self.tts_model_dir, self.tts_config_file_name))
        self.tts_model = Xtts.init_from_config(config)
        self.tts_model.load_checkpoint(config, checkpoint_dir=self.tts_model_dir, use_deepspeed=self.tts_use_deepspeed)

        if self.tts_use_cuda and torch.cuda.is_available():
            self.tts_model.cuda()
        else:
            rospy.logwarn("CUDA is not available, using CPU!")
            self.tts_model.cpu()
                
    def __read_params(self):
        self.robot_name = rospy.get_param("robot_name", "BORIS").lower()

        self.speech_synthesizer_service_name = rospy.get_param("services/speech_synthesizer/service", "/butia_speech/ss/say_something")
        self.speech_synthesizer_topic_name = rospy.get_param("subscribers/speech_synthesizer/topic", "/butia_speech/ss/say_something")
        self.audio_player_service_name = rospy.get_param("services/audio_player/service", "/butia_speech/ap/audio_player")

        self.tts_model_name = rospy.get_param("tts/model_name", "tts_models/multilingual/multi-dataset/xtts_v2")
        self.tts_config_file_name = rospy.get_param("tts/config_file_name", "config.json")
        self.tts_use_deepspeed = rospy.get_param("tts/use_deepspeed", True)
        self.tts_use_cuda = rospy.get_param("tts/use_cuda", True)
        self.tts_temperature = rospy.get_param("tts/temperature", 0.7)

    def __init_comm(self):
        self.speech_synthesizer_server = rospy.Service("/butia_speech/ss/say_something", SynthesizeSpeech, self.synthesize_speech)
        self.speech_synthesizer_subscriber = rospy.Subscriber("/butia_speech/ss/say_something", SynthesizeSpeechMessage, self.synthesize_speech)
    
    def __compute_voice_latents(self):
        robot_voices_dir = os.path.join(VOICES_DIR, self.robot_name)
        if not os.path.exists(robot_voices_dir):
            rospy.logerr("Robot voices directory not found: " + robot_voices_dir)
            exit()
        
        robot_voices_files = [ os.path.join(robot_voices_dir, filename) for filename in os.listdir(robot_voices_dir) if filename.endswith(WAV_EXTENSION) ]

        gpt_cond_latent, speaker_embedding = self.tts_model.get_conditioning_latents(audio_path=robot_voices_files)

        self.voice_latents = { "gpt_cond_latent": gpt_cond_latent, "speaker_embedding": speaker_embedding }

    def synthesize_speech(self, req):
        text = req.text
        lang = req.lang if req.lang else "en"

        rospy.logdebug("Synthesizing speech: " + text)
        rospy.logdebug("Language: " + lang)

        out = self.tts_model.inference(
            text,
            lang,
            self.voice_latents['gpt_cond_latent'],
            self.voice_latents['speaker_embedding'],
            temperature=self.tts_temperature,
        )
        wav = torch.tensor(out['wav'])
        print(wav.shape, wav.dtype, wav)

        wavfile.write(FILENAME, self.sample_rate, (wav.view(-1).cpu().numpy()*32768).astype(np.int16))
        
        rospy.wait_for_service(self.audio_player_service_name, timeout=rospy.Duration(10))
        try:
            audio_player = rospy.ServiceProxy(self.audio_player_service_name, AudioPlayer)
            audio_player(FILENAME)
        
            return SynthesizeSpeechResponse(Bool(True))
        except rospy.ServiceException as exc:
            rospy.logerr("Service call failed: %s" % exc)
            return SynthesizeSpeechResponse(Bool(False))

        need_response = True if isinstance(req, SynthesizeSpeechRequest) else False
        
if __name__ == '__main__':
    XTTSSpeechSynthesizerNode()
    rospy.spin()
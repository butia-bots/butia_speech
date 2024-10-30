import audioop
import os
import wave
import rospy
import rospkg
import numpy as np
import struct
import sounddevice as sd
from std_msgs.msg import Int16MultiArray, Int16
from threading import Lock, Event
from scipy import signal
import fractions

BUTIA_SPEECH_PKG = rospkg.RosPack().get_path("butia_speech")
AUDIO = os.path.join(BUTIA_SPEECH_PKG, "audios/")

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

class WavToMouth():

    def __init__(self):
        self.chunk_size = rospy.get_param("chunk_size", 2048)

        self.data = []
        self.data_lock = Lock()  # To ensure thread-safe access to self.data

        self.max = 0
        self.min = 0

        self.audio = None
        self.audio_info = None

        self.output = Int16MultiArray()
        self.mouth_gain = rospy.get_param("mouth_gain", 2.5)
        self.angle_publisher = rospy.Publisher("mouth", Int16MultiArray, queue_size=1)
        self.mouth_debug_publisher = rospy.Publisher("mouth_debug", Int16, queue_size=1)

        self.streaming = False
        self.request_stream_stop = False

        self.sample_rate = 44100  # Default value; will be updated
        self.channels = 1         # Default value; will be updated
        
        self.playback_speed = rospy.get_param("playback_speed", 1.0)

        self.stream = None
        self.stream_lock = Lock()  # To manage access to the stream

        self.playback_done_event = Event()  # Event to signal playback completion

    def divide_audio_in_chunks(self, audio_data, num_bytes=2):
        chunks = []
        for start in range(0, len(audio_data), self.chunk_size*num_bytes):
            end = min(start + self.chunk_size*num_bytes, len(audio_data))
            chunks.append(audio_data[start:end])
        return chunks

    def _read_data_of_audio(self):
        self.audio = wave.open(self.filepath, "rb")
        audio_data = self.audio.readframes(self.audio.getnframes())
        self.data += self.divide_audio_in_chunks(audio_data, num_bytes=self.audio.getsampwidth())
        self.sample_rate = self.audio.getframerate()
        self.channels = self.audio.getnchannels()

    def request_stop_stream(self):
        self.request_stream_stop = True

    def start_stream(self):
        with self.stream_lock:
            if self.streaming:
                rospy.logwarn("Stream is already running.")
                return
            self.streaming = True
            self.request_stream_stop = False

            if self.stream is None:
                self.stream = sd.OutputStream(
                    samplerate=int(self.sample_rate * self.playback_speed),  # Adjusted sample rate for speed
                    channels=self.channels,
                    callback=self.audio_callback,
                    blocksize=self.chunk_size,
                    dtype='int16'
                )
                self.stream.start()
                rospy.loginfo("Audio stream started.")
            else:
                rospy.logwarn("Stream already initialized.")

    def stop_stream(self):
        with self.stream_lock:
            if not self.streaming:
                rospy.logwarn("Stream is not running.")
                return
            self.streaming = False
            self.output.data = [0, 100]
            self.angle_publisher.publish(self.output)
            if self.stream is not None:
                self.stream.stop()
                self.stream.close()
                self.stream = None
                rospy.loginfo("Audio stream stopped.")
            sd.stop()

    def set_audio_info(self, audio_info):
        self.audio_info = audio_info
        self._open_stream()

    def set_filepath(self, filepath):
        self.filepath = os.path.join(AUDIO, filepath)
        self._read_data_of_audio()

    def set_data_and_info(self, data, info):
        self.data += self.divide_audio_in_chunks(data)
        self.audio_info = info
        self._open_stream()

    def _open_stream(self):
        if self.audio is not None:
            self.sample_rate = self.audio.getframerate()
            self.channels = self.audio.getnchannels()
        elif self.audio_info is not None:
            self.sample_rate = self.audio_info.sample_rate
            self.channels = self.audio_info.channels

        self.audio = None
        self.audio_info = None

    def _compute_chunk_rms(self, audio_chunk):
        """Compute the RMS value of an audio chunk."""
        count = len(audio_chunk) // 2
        format = "%dh" % count
        shorts = struct.unpack(format, audio_chunk)
        sum_squares = sum(s ** 2 for s in shorts)
        rms = np.sqrt(sum_squares / count)
        return rms

    def _normalize_rms(self, rms_value, gain=2.5, max_rms=32768):
        """Normalize the RMS value to a range from 0 to 100."""
        rms_value = min(rms_value * gain, max_rms)
        return min(max(int((rms_value / max_rms) * 100), 0), 100)

    def stream_data_callback(self, data):
        with self.data_lock:
            self.data += self.divide_audio_in_chunks(data)

    def audio_callback(self, outdata, frames, time, status):
        if status:
            rospy.logerr(f"Stream status: {status}")
        with self.data_lock:
            if len(self.data) == 0:
                # If no data is available, output silence
                outdata[:] = np.zeros((frames, self.channels), dtype='int16')
                # Signal that playback is done
                self.playback_done_event.set()
                return

            # Get the next chunk
            data = self.data.pop(0)

        # Convert bytes to NumPy array
        audio_array = np.frombuffer(data, dtype=np.int16)

        # Modify playback speed using interpolation
        if self.playback_speed != 1.0:
            original_indices = np.arange(audio_array.shape[0])
            new_length = int(audio_array.shape[0] / self.playback_speed)
            new_indices = np.linspace(0, original_indices[-1], new_length)
            
            # Interpolate audio data to fit new indices
            audio_array = np.interp(new_indices, original_indices, audio_array).astype(np.int16)

        # Reshape based on channels
        if self.channels > 1:
            try:
                audio_array = audio_array.reshape(-1, self.channels)
            except ValueError:
                rospy.logerr("Audio data size is not compatible with the number of channels.")
                audio_array = np.zeros((frames, self.channels), dtype='int16')

        # Handle cases where the chunk size doesn't match the stream's blocksize
        if len(audio_array) < frames * self.channels:
            # Pad with zeros if the chunk is smaller
            pad_width = (frames * self.channels) - len(audio_array)
            audio_array = np.pad(audio_array, (0, pad_width), 'constant', constant_values=0)
        elif len(audio_array) > frames * self.channels:
            # Trim the chunk if it's larger
            audio_array = audio_array[:frames * self.channels]

        outdata[:] = audio_array.reshape((frames, self.channels))

        # Compute RMS and publish mouth angles
        rms = self._compute_chunk_rms(data)
        mouth_angle = self._normalize_rms(rms, gain=self.mouth_gain)
        self.output.data = [mouth_angle, abs(100 - mouth_angle)]
        self.angle_publisher.publish(self.output)
        self.mouth_debug_publisher.publish(Int16(mouth_angle))

    def play_all_data(self):
        # Clear the playback done event
        self.playback_done_event.clear()
        # Start the audio stream
        self.start_stream()
        # Wait until the playback is done
        self.playback_done_event.wait()
        # Stop the stream gracefully
        self.stop_stream()

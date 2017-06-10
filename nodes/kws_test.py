#!/usr/bin/python

import os

import pyaudio

import rospy
import rospkg

from std_msgs.msg import String
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *


# Class to add keyword spotting functionality
class KWSDetection(object):

    def __init__(self):

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("kws_data", String, queue_size=10)
        # Initalizing publisher for continuous ASR
        self.continuous_pub_ = rospy.Publisher(
            "jsgf_audio", String, queue_size=10)

        # initialize node
        rospy.init_node("kws_control")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)
        # Initializing rospack to find package location
        rospack = rospkg.RosPack()

        # Params

        # Location of external files
        self.location = rospack.get_path('pocketsphinx') + '/demo/'
        # File containing language model
        self._lm_param = "~lm"
        # Dictionary
        self._dict_param = "~dict"
        # List of keywords to detect
        self._kws_param = "~kws"
        # Not necessary to provide the next two if _kws_param is provided
        # Single word which needs to be detected
        self._keyphrase_param = "~keyphrase"
        # Threshold frequency of above mentioned word
        self._threshold_param = "~threshold"
        # Option for continuous
        self._option_param = "~option"

        # Variable to distinguish between kws list and keyphrase.
        # Default is keyword list
        self._list = True
        # For continuous mode
        self.stop_output = False

        # Setting param values
        if rospy.has_param(self._lm_param):
            self.lm = self.location + rospy.get_param(self._lm_param)
            if rospy.get_param(self._lm_param) == ":default":
                if (os.path.isdir("/usr/local/share/pocketsphinx/model")):
                    rospy.loginfo("Loading the default acoustic model")
                    self.lm = "/usr/local/share/pocketsphinx/model/en-us/en-us"
                    rospy.loginfo("Done loading the default acoustic model")
                else:
                    rospy.logerr(
                        "No language model specified. Couldn't find defaut model.")
                    return
        else:
            rospy.loginfo("Couldn't find lm argument")

        if rospy.has_param(self._dict_param) and rospy.get_param(self._dict_param) != ":default":
            self.lexicon = self.location + rospy.get_param(self._dict_param)
        else:
            rospy.logerr(
                'No dictionary found. Please add an appropriate dictionary argument.')
            return
        rospy.loginfo(rospy.get_param(self._kws_param))

        if rospy.has_param(self._kws_param) and rospy.get_param(self._kws_param) != ":default":
            self._list = True

            self.kw_list = self.location + rospy.get_param(self._kws_param)
        elif rospy.has_param(self._keyphrase_param) and rospy.has_param(self._threshold_param) and rospy.get_param(self._keyphrase_param) != ":default" and rospy.get_param(self._threshold_param) != ":default":
            self._list = False

            self.keyphrase = rospy.get_param(self._keyphrase_param)
            self.kws_threshold = rospy.get_param(self._threshold_param)
        else:
            rospy.logerr(
                'kws cant run. Please add an appropriate keyword list.')
            return

        # All params satisfied. Starting recognizer
        self.start_recognizer()

    # Function to handle keyword spotting of audio
    def start_recognizer(self):

        config = Decoder.default_config()
        rospy.loginfo("Pocketsphinx initialized")

        # Setting configuration of decoder using provided params
        config.set_string('-hmm', self.lm)
        config.set_string('-dict', self.lexicon)

        if self._list:
            # Keyword list file for keyword searching
            config.set_string('-kws', self.kw_list)
        else:
            # In case keyphrase is provided
            config.set_string('-keyphrase', self.keyphrase)
            config.set_float('-kws_threshold', self.kws_threshold)

        # Set required configuration for decoder
        self.decoder = Decoder(config)

        # Start processing input audio
        self.decoder.start_utt()
        rospy.loginfo("Decoder started successfully")

        # Subscribe to audio topic
        rospy.Subscriber("sphinx_msg", String, self.process_audio)
        rospy.spin()

    # Audio processing based on decoder config
    def process_audio(self, data):

        # Check if keyword detected
        if !self.stop_output:
            if self.decoder.hyp() != None:
                # Actual processing
                self.decoder.process_raw(data.data, False, False)
                rospy.loginfo([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                               for seg in self.decoder.seg()])
                rospy.loginfo("Detected keyphrase, restarting search")
                seg.word = seg.word.lower()
                self.decoder.end_utt()
                # Publish output to a topic
                self.pub_.publish(seg.word)
                if rospy.has_param(self._option_param):
                    self.stop_output = True
                else:
                    self.decoder.start_utt()
        else:
            self.continuous_pub_.publish(data.data)

    def shutdown(self):
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop ASRControl")
        self.pub_.publish("")
        rospy.sleep(1)


if __name__ == "__main__":
    start = KWSDetection()

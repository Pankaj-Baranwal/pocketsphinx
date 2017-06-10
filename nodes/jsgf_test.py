#!/usr/bin/python

import os

import pyaudio

import rospy
import rospkg

from std_msgs.msg import String
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *


# Class to add jsgf grammar functionality
class JSGFTest(object):

    def __init__(self):

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("grammar_data", String, queue_size=10)

        # initialize node
        rospy.init_node("jsgf_control")
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
        # Hidden markov model. Default has been provided below
        self._hmm_param = "~hmm"
        # Gram file contains the rules and grammar
        self._gram = "~gram"
        # Name of rule within the grammar
        self._rule = "~rule"

        # check if lm or grammar mode. Default = grammar
        self._use_lm = 0

        # Setting param values
        if rospy.has_param(self._hmm_param):
            self.hmm = self.location + rospy.get_param(self._hmm_param)
            if rospy.get_param(self._hmm_param) == ":default":
                if os.path.isdir("/usr/local/share/pocketsphinx/model"):
                    rospy.loginfo("Loading the default acoustic model")
                    self.hmm = "/usr/local/share/pocketsphinx/model/en-us/en-us"
                    rospy.loginfo("Done loading the default acoustic model")
                else:
                    rospy.logerr(
                        "No language model specified. Couldn't find default model.")
                    return
        else:
            rospy.logerr(
                "No language model specified. Couldn't find default model.")
            return

        if rospy.has_param(self._dict_param) and rospy.get_param(self._dict_param) != ":default":
            self.dict = self.location + rospy.get_param(self._dict_param)
        else:
            rospy.logerr(
                "No dictionary found. Please add an appropriate dictionary argument.")
            return

        if rospy.has_param(self._lm_param) and rospy.get_param(self._lm_param) != ':default':
            self._use_lm = 1
            self.lm = self.location + rospy.get_param(self._lm_param)
        elif rospy.has_param(self._gram) and rospy.has_param(self._rule):
            self._use_lm = 0
            self.gram = rospy.get_param(self._gram)
            self.rule = rospy.get_param(self._rule)
        else:
            rospy.logerr(
                "Couldn't find suitable parameters. Please take a look at the documentation")
            return

        # All params satisfied. Starting recognizer
        self.start_recognizer()

    # Function to handle lm or grammar processing of audio
    def start_recognizer(self):

        config = Decoder.default_config()
        rospy.loginfo("Done initializing pocketsphinx")

        # Setting configuration of decoder using provided params
        config.set_string('-hmm', self.hmm)
        config.set_string('-dict', self.dict)

        # Check if language model to be used or grammar mode
        if (self._use_lm):
            rospy.loginfo('Language Model Found.')
            self.mode = "lanugage model: "
            config.set_string('-lm', self.lm)
            self.decoder = Decoder(config)
        else:
            rospy.loginfo(
                'language model not found. Using JSGF grammar instead.')
            self.decoder = Decoder(config)
            self.mode = "grammar: "

            # Switch to JSGF grammar
            jsgf = Jsgf(self.location + self.gram + '.gram')
            rule = jsgf.get_rule(self.gram + '.' + self.rule)
            # Using finite state grammar as mentioned in the rule
            fsg = jsgf.build_fsg(rule, self.decoder.get_logmath(), 7.5)
            rospy.loginfo("Writing fsg to " +
                          self.location + self.gram + '.fsg')
            fsg.writefile(self.location + self.gram + '.fsg')

            self.decoder.set_fsg(self.gram, fsg)
            self.decoder.set_search(self.gram)

        # Start processing input audio
        self.decoder.start_utt()
        rospy.loginfo("Decoder started successfully")

        # Subscribe to audio topic
        rospy.Subscriber("sphinx_msg", String, self.process_audio)
        rospy.spin()

    # Audio processing based on decoder config
    def process_audio(self, data):
        # Check if input audio has ended
        if data.data == "ended":
            self.decoder.end_utt()
            rospy.loginfo('Decoding with ' + self.mode +
                          'produced output: ' + self.decoder.hyp().hypstr)
            # Publish output to a topic
            self.pub_.publish(self.decoder.hyp().hypstr)
        else:
            # Actual processing
            self.decoder.process_raw(data.data, False, False)

    def shutdown(self):
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop ASRControl")
        self.pub_.publish("")
        rospy.sleep(1)


if __name__ == "__main__":
    JSGFTest()

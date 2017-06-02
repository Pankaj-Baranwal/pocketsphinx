#!/usr/bin/python

import sys, os

import argparse
import rospy

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio

from std_msgs.msg import String
from std_srvs.srv import *
from os import path
import commands

class KWSDetection(object):
    def __init__(self):
        # Start node
        rospy.init_node("kws_control")
        rospy.on_shutdown(self.shutdown)

        # Params
        self._lm_param = "~lm"
        self._dict_param = "~dict"
        self._kws_param = "~kws"
        # Not necessary to provide the next two if _kws_param is provided
        self._keyphrase_param = "~keyphrase"
        self._threshold_param = "~threshold"

        # Variable to distinguis between kws list and keyphrase
        self._list = True;


        self.pub_ = rospy.Publisher("kws_data", String, queue_size=10)

        # Check if required arguments provided in command line
        if rospy.has_param(self._lm_param):
            self.lm = rospy.get_param(self._lm_param)
        else:
            if (os.path.isdir("/usr/local/share/pocketsphinx/model")):
                rospy.loginfo("Loading the default acoustic model")
                self.lm = "/usr/local/share/pocketsphinx/model/en-us/en-us"
                rospy.loginfo("Done loading the default acoustic model")
            else:
                rospy.logerr("No language model specified. Couldn't find defaut model.")
                return

        if rospy.has_param(self._dict_param):
            self.lexicon = rospy.get_param(self._dict_param)
        else:
            rospy.logerr('No dictionary found. Please add an appropriate dictionary argument.')
            return

        if rospy.has_param(self._kws_param):
            self._list = True;

            self.kw_list = rospy.get_param(self._kws_param)
        else if rospy.has_param(self._keyphrase_param) and rospy.has_param(self._threshold_param):
            self._list = False;

            self.keyphrase = rospy.get_param(self._keyphrase_param)
            self.kws_threshold = rospy.get_param(self._threshold_param)
        else:
            rospy.logerr('kws cant run. Please add an appropriate keyword list.')
            return

        # If requirements fulfilled, start recognizer
        self.start_recognizer()

    def start_recognizer(self):
        # initialize pocketsphinx.
        rospy.loginfo("Initializing pocketsphinx")
        config = Decoder.default_config()
        rospy.loginfo("Done initializing pocketsphinx")

        # Hidden Markov model: The model which has been used
        config.set_string('-hmm', self.lm)
        #Pronunciation dictionary used
        config.set_string('-dict', self.lexicon)

        if self._list:
            # Keyword list file for keyword searching
            config.set_string('-kws', self.kw_list)
        else:
            # In case keyphrase is provided
            config.set_string('-keyphrase', self.keyphrase)
            config.set_float('-kws_threshold', self.kws_threshold)

        rospy.loginfo("Opening the audio channel")
        # I recommend installing and running audacity to help figure this out
        # Other useful commands:
        # pactl list short sources
        # pacmd list-sinks
        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=1024)
        stream.start_stream()
        rospy.loginfo("Done opening the audio channel")

        #decoder streaming data
        rospy.loginfo("Starting the decoder")
        self.decoder = Decoder(config)
        self.decoder.start_utt()
        rospy.loginfo("Done starting the decoder")

        while not rospy.is_shutdown():
            # taken as is from python wrapper
            buf = stream.read(1024)
            if buf:
                self.decoder.process_raw(buf, False, False)
            else:
                break
            self.parse_asr_result()

    def parse_asr_result(self):
        """
        move the robot based on ASR hypothesis
        """
        if self.decoder.hyp() != None:
            rospy.loginfo([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                for seg in self.decoder.seg()])
            rospy.loginfo("Detected keyphrase, restarting search")
            seg.word = seg.word.lower()
            self.decoder.end_utt()
            self.decoder.start_utt()
            self.pub_.publish(seg.word)

    def shutdown(self):
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop ASRControl")
        self.pub_.publish("")
        rospy.sleep(1)


if __name__ == "__main__":
    if len(sys.argv) > 0:
        start = KWSDetection()
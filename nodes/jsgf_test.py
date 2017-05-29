#!/usr/bin/python

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

import argparse
import rospy

import pyaudio

from std_msgs.msg import String
from std_srvs.srv import *
from os import environ, path
import commands

MODELDIR = "../../../model"
DATADIR = "../../../test/data"

class JSGFTest(object):
    def __init__(self):
        # Start node
        rospy.init_node("jsgf_control")
        rospy.on_shutdown(self.shutdown)

        self._use_lm = 1

        # Params
        self._hmm_param = "~hmm"
        self._lm_param = "~lm"
        self._dict_param = "~dict"
        self._audio = "~input"
        self._gram = "~gram"

        self.pub_ = rospy.Publisher("lm_data", String, queue_size=10)

        if rospy.has_param(self._audio):
            self.audio = rospy.get_param(self._audio)
        else:
            self.audio = path.join(DATADIR, 'goforward.raw')
            rospy.loginfo("Loading default goforward audio input")

        if rospy.has_param(self._hmm_param):
            self.hmm = rospy.get_param(self._hmm_param)
        else:
            self.hmm = path.join(MODELDIR, 'en-us/en-us')
            rospy.loginfo("Loading the by default hidden markov model")

        if rospy.has_param(self._lm_param):
            self.lm = rospy.get_param(self._lm_param)
        else:
            self._use_lm = 0
            # self.lm = path.join(DATADIR, 'turtle.lm.bin')
            # rospy.loginfo("Loading the turtle language model by default")

        if rospy.has_param(self._dict_param):
            self.dict = rospy.get_param(self._dict_param)
        else:
            self.dict = path.join(DATADIR, 'turtle.dic')
            rospy.loginfo("Loading the turtle dictionary by default")

        self.start_recognizer()

    def start_recognizer(self):
        rospy.loginfo("Initializing pocketsphinx")
        # Create a decoder with certain model
        config = Decoder.default_config()
        rospy.loginfo("Done initializing pocketsphinx")

        config.set_string('-hmm', self.hmm)
        if (self._use_lm):
            config.set_string('-lm', self.lm)
        config.set_string('-dict', self.dict)
        decoder = Decoder(config)

        if (self._use_lm):
            print ('Language Model Found.')
            # Decode with lm
            decoder.start_utt()
            # To convert wav file into raw audio format, install sox.
            # Terminal command: sox <wav file name> <target raw audio file>
            stream = open(self.audio, 'rb')
            # stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
            #                         rate=16000, input=True, frames_per_buffer=1024)
            while True:
                buf = stream.read(1024)
                if buf:
                     decoder.process_raw(buf, False, False)
                else:
                     break
            decoder.end_utt()
            print ('Decoding with the provided language:', decoder.hyp().hypstr)
            self.pub_.publish(decoder.hyp().hypstr)
        else:
            print ('language model not found. Using JSGF grammar instead.')
            # Switch to JSGF grammar
            if rospy.has_param(self._gram):
                self.gram = rospy.get_param(self._gram)
                print ('Using provided gram file')
            else:
                print ('Loading default gram file')
                self.gram = path.join(DATADIR, 'goforward.gram')
            jsgf = Jsgf((self.gram + '.gram'))
            if rospy.has_param(self._rule):
                self.rule = rospy.get_param(self._rule)
                print ('Using provided rule')
            else:
                print ('Loading default rule')
                self.rule = 'move2'
            rule = jsgf.get_rule((self.gram + '.' + self.rule))
            fsg = jsgf.build_fsg(rule, decoder.get_logmath(), 7.5)
            fsg.writefile((self.gram + '.fsg'))

            decoder.set_search(self.gram)
            decoder.set_fsg(self.gram, fsg)

            decoder.start_utt()
            stream = open(self.audio, 'rb')
            # stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
            #                         rate=16000, input=True, frames_per_buffer=1024)
            while True:
                buf = stream.read(1024)
                if buf:
                     decoder.process_raw(buf, False, False)
                else:
                     break
            decoder.end_utt()
            print ('Decoding with grammar:', decoder.hyp().hypstr)
            self.pub_.publish(decoder.hyp().hypstr)

if __name__ == "__main__":
    if len(sys.argv) > 0:
        start = JSGFTest()
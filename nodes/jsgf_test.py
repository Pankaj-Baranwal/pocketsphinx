#!/usr/bin/python

import os, sys
import rospy

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

import pyaudio

from std_msgs.msg import String
from std_srvs.srv import *
from os import path
import commands

class JSGFTest(object):
    def __init__(self):
        # Start node
        rospy.init_node("jsgf_control")
        rospy.on_shutdown(self.shutdown)

        # Params
        self._audio = "~input"
        self._lm_param = "~lm"
        self._dict_param = "~dict"
        self._hmm_param = "~hmm"
        self._gram = "~gram"
        self._rule = "~rule"

        # check if user wants lm or grammar mode
        self._use_lm = 1
        self._use_microphone = False

        self.pub_ = rospy.Publisher("lm_data", String, queue_size=10)

        if rospy.has_param(self._audio):
            self._use_microphone = False
            self.audio = rospy.get_param(self._audio)
        else:
            self._use_microphone = True
            rospy.loginfo('No audio file provided. Using default microphone instead')

        if rospy.has_param(self._hmm_param):
            self.hmm = rospy.get_param(self._hmm_param)
        else if (os.path.isdir("/usr/local/share/pocketsphinx/model")):
            rospy.loginfo("Loading the default acoustic model")
            self.hmm = "/usr/local/share/pocketsphinx/model/en-us/en-us"
            rospy.loginfo("Done loading the default acoustic model")
        else:
            rospy.logerr("No language model specified. Couldn't find default model.")
            return

        if rospy.has_param(self._dict_param):
            self.dict = rospy.get_param(self._dict_param)
        else:
            rospy.logerr("No dictionary found. Please add an appropriate dictionary argument.")
            return

        if rospy.has_param(self._lm_param):
            self._use_lm = 1
            self.lm = rospy.get_param(self._lm_param)
        else if rospy.has_param(self._gram) and rospy.has_param(self._rule):
            self._use_lm = 0
            self.gram = rospy.get_param(self._gram)
            self.rule = rospy.get_param(self._rule)
        else:
            rospy.logerr("Couldn't find suitable parameters. Please take a look at the documentation")
            return

        self.start_recognizer()

    def start_recognizer(self):
        rospy.loginfo("Initializing pocketsphinx")
        config = Decoder.default_config()
        rospy.loginfo("Done initializing pocketsphinx")

        config.set_string('-hmm', self.hmm)
        config.set_string('-dict', self.dict)
        if (self._use_lm):
            config.set_string('-lm', self.lm)
        decoder = Decoder(config)

        if (self._use_lm):
            rospy.loginfo('Language Model Found.')
            # Decode with lm
            process_audio(decoder, 'language model: ')
        else:
            rospy.loginfo('language model not found. Using JSGF grammar instead.')
            # Switch to JSGF grammar
            jsgf = Jsgf((self.gram + '.gram'))
            rule = jsgf.get_rule((self.gram + '.' + self.rule))
            
            fsg = jsgf.build_fsg(rule, decoder.get_logmath(), 7.5)
            fsg.writefile((self.gram + '.fsg'))

            decoder.set_search(self.gram)
            decoder.set_fsg(self.gram, fsg)

            process_audio(decoder, 'grammar: ')

    def  process_audio(decoder, message):
        decoder.start_utt()

        stream_init(self)
        
        while True:
            buf = self.stream.read(1024)
            if buf:
                 decoder.process_raw(buf, False, False)
            else:
                 break
        decoder.end_utt()
        print ('Decoding with ', message, decoder.hyp().hypstr)
        self.pub_.publish(decoder.hyp().hypstr)

    def stream_init(self):
        if self._use_microphone:
            self.stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                                    rate=16000, input=True, frames_per_buffer=1024)
        else:
            # To convert wav file into raw audio format, install sox.
            # Command for conversion: sox <wav file name> <target raw audio file>
            self.stream = open(self.audio, 'rb')

if __name__ == "__main__":
    if len(sys.argv) > 0:
        start = JSGFTest()
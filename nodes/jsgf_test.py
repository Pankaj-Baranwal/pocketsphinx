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

        self.pub_ = rospy.Publisher("grammar_data", String, queue_size=10)

        # Start node
        rospy.init_node("jsgf_control")
        rospy.on_shutdown(self.shutdown)

        # Params
        self._lm_param = "~lm"
        self._dict_param = "~dict"
        self._hmm_param = "~hmm"
        self._gram = "~gram"
        self._rule = "~rule"

        # check if user wants lm or grammar mode
        self._use_lm = 1

        if rospy.has_param(self._hmm_param):
            self.hmm = rospy.get_param(self._hmm_param)
        elif (os.path.isdir("/usr/local/share/pocketsphinx/model")):
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
            rospy.loginfo(self._lm_param)
            rospy.loginfo(rospy.get_param(self._lm_param))
            self._use_lm = 1
            self.lm = rospy.get_param(self._lm_param)
        elif rospy.has_param(self._gram) and rospy.has_param(self._rule):
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
            rospy.loginfo('Language Model Found.')
            self.mode = "lanugage model: "
            config.set_string('-lm', self.lm)
            self.decoder = Decoder(config)
        else:
            self.decoder = Decoder(config)
            self.mode = "grammar: "
            rospy.loginfo('language model not found. Using JSGF grammar instead.')
            # Switch to JSGF grammar
            jsgf = Jsgf((self.gram + '.gram'))
            rule = jsgf.get_rule((self.gram + '.' + self.rule))
            
            fsg = jsgf.build_fsg(rule, self.decoder.get_logmath(), 7.5)
            fsg.writefile((self.gram + '.fsg'))

            self.decoder.set_fsg(self.gram, fsg)

            self.decoder.set_search(self.gram)

        self.decoder.start_utt()

        rospy.Subscriber("sphinx_msg", String, self.process_audio)
        rospy.spin()

    def  process_audio(self, data):
        if data.data == "ended":
            self.decoder.end_utt()  
            # rospy.loginfo('Decoding with ', self.mode, self.decoder.hyp().hypstr)
            self.pub_.publish(self.decoder.hyp().hypstr)
        else:
            self.decoder.process_raw(data.data, False, False)
        # if self.decoder.hyp() != None: 
        # rospy.loginfo('Decoding with ', self.mode, self.decoder.hyp().hypstr)

    def shutdown(self):
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop ASRControl")
        self.pub_.publish("")
        rospy.sleep(1)

if __name__ == "__main__":
    if len(sys.argv) > 0:
        start = JSGFTest()
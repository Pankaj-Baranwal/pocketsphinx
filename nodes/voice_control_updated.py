#!/usr/bin/env python

import argparse
import rospy

from geometry_msgs.msg import Twist

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio

from std_msgs.msg import String
from std_srvs.srv import *
import os
import commands

class ASRControl(object):

    def __init__(self):

        # initialize ROS
        self.speed = 0.2
        self.msg = Twist()

        # Start node
        rospy.init_node("asr_control")
        rospy.on_shutdown(self.shutdown)

        self._lm_param = "~lm"
        self._dict_param = "~dict"
        self._kws_param = "~kws"

        # you may need to change publisher destination depending on what you run
        self.pub_ = rospy.Publisher("voice_data", Twist, queue_size=10)

        if rospy.has_param(self._lm_param):
            self.lm = rospy.get_param(self._lm_param)
        else:
            rospy.loginfo("Loading the default acoustic model")
            self.lm = "/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k"
            rospy.loginfo("Done loading the default acoustic model")

        if rospy.has_param(self._dict_param):
            self.lexicon = rospy.get_param(self._dict_param)
        else:
            rospy.logerr('No dictionary found. Please add an appropriate dictionary argument.')
            return

        if rospy.has_param(self._kws_param):
            self.kw_list = rospy.get_param(self._kws_param)
        else:
            rospy.logerr('kws cant run. Please add an appropriate keyword list file.')
            return
        self.start_recognizer()

    def start_recognizer(self):
        # initialize pocketsphinx. As mentioned in python wrapper
        rospy.loginfo("Initializing pocketsphinx")
        config = Decoder.default_config()
        rospy.loginfo("Done initializing pocketsphinx")

        # Hidden Markov model: The model which has been used
        config.set_string('-hmm', self.lm)
        #Pronunciation dictionary used
        config.set_string('-dict', self.lexicon)
        #Keyword list file for keyword searching
        config.set_string('-kws', self.kw_list)

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
            print ([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                for seg in self.decoder.seg()])
            print ("Detected keyphrase, restarting search")
            seg.word = seg.word.lower()
            self.decoder.end_utt()
            self.decoder.start_utt()
            # you may want to modify the main logic here
            if seg.word.find("full speed") > -1:
                if self.speed == 0.2:
                    self.msg.linear.x = self.msg.linear.x*2
                    self.msg.angular.z = self.msg.angular.z*2
                    self.speed = 0.4
            if seg.word.find("half speed") > -1:
                if self.speed == 0.4:
                    self.msg.linear.x = self.msg.linear.x/2
                    self.msg.angular.z = self.msg.angular.z/2
                    self.speed = 0.2
            if seg.word.find("forward") > -1:
                self.msg.linear.x = self.speed
                self.msg.angular.z = 0
            elif seg.word.find("left") > -1:
                if self.msg.linear.x != 0:
                    if self.msg.angular.z < self.speed:
                        self.msg.angular.z += 0.05
                else:
                    self.msg.angular.z = self.speed*2
            elif seg.word.find("right") > -1:
                if self.msg.linear.x != 0:
                    if self.msg.angular.z > -self.speed:
                        self.msg.angular.z -= 0.05
                else:
                    self.msg.angular.z = -self.speed*2
            elif seg.word.find("back") > -1:
                self.msg.linear.x = -self.speed
                self.msg.angular.z = 0
            elif seg.word.find("stop") > -1 or seg.word.find("halt") > -1:
                self.msg = Twist()

        self.pub_.publish(self.msg)

    def shutdown(self):
        """
        command executed after Ctrl+C is pressed
        """
        rospy.loginfo("Stop ASRControl")
        self.pub_.publish(Twist())
        rospy.sleep(1)



if __name__ == "__main__":
    if len(sys.argv) > 0:
        start = ASRControl()


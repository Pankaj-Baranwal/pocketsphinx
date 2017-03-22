#!/usr/bin/env python

import argparse
import roslib
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
        self.pub_ = rospy.Publisher(pub, Twist, queue_size=10)

        if rospy.has_param(self._lm_param):
            lm = rospy.get_param(self._lm_param)
        else:
            rospy.logerr('Recognizer not started. Please specify a language model file.')
        return

        if rospy.has_param(self._dict_param):
            dic = rospy.get_param(self._dict_param)
        else:
            rospy.logerr('No dictionary found. Please add an appropriate dictionary argument.')
        return

        if rospy.has_param(self._kws_param):
            kws = rospy.get_param(self._kws_param)
        else:
            rospy.logerr('kws cant run. Please add an appropriate keyword list file.')
        return



        # initialize pocketsphinx. As mentioned in python wrapper
        config = Decoder.default_config()

        # Hidden Markov model: The model which has been used
        config.set_string('-hmm', model)
        #Pronunciation dictionary used
        config.set_string('-dict', lexicon)
        #Keyword list file for keyword searching
        config.set_string('-kws', kwlist)

        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=1024)


        rospy.loginfo("Launch config: %s", self.launch_config)
        
        # This is for GStreamer. But What does this do? Is it configuration of GStreamer?
        self.launch_config += " ! audioconvert ! audioresample " \
                            + '! vader name=vad auto-threshold=true ' \
                            + '! pocketsphinx name=asr ! fakesink'

        # Configure ROS settings
        self.started = False
        rospy.on_shutdown(self.shutdown)
        self.pub = rospy.Publisher('~output', String)
        rospy.Service("~start", Empty, self.start)
        rospy.Service("~stop", Empty, self.stop)

        if rospy.has_param(self._lm_param) and rospy.has_param(self._dic_param):
            self.start_recognizer()
        else:
            rospy.logwarn("lm and dic parameters need to be set to start recognizer.")

    def start_recognizer(self):
        rospy.loginfo("Starting recognizer... ")
        
        # ASR = Automatic Speech Recognition
        self.pipeline = gst.parse_launch(self.launch_config)
        #name has been mentioned in launch_config
        self.asr = self.pipeline.get_by_name('asr')
        self.asr.connect('partial_result', self.asr_partial_result)
        self.asr.connect('result', self.asr_result)
        self.asr.set_property('configured', True)
        # dsratio?
        self.asr.set_property('dsratio', 1)

        # Configure language model
        if rospy.has_param(self._lm_param):
            lm = rospy.get_param(self._lm_param)
        else:
            rospy.logerr('Recognizer not started. Please specify a language model file.')
            return

        if rospy.has_param(self._dic_param):
            dic = rospy.get_param(self._dic_param)
        else:
            rospy.logerr('Recognizer not started. Please specify a dictionary.')
            return

        self.asr.set_property('lm', lm)
        self.asr.set_property('dict', dic)
        
        """
        A bus is a simple system that takes care of forwarding messages 
        from the streaming threads to an application in its own thread context. 
        The advantage of a bus is that an application does not need to be 
        thread-aware in order to use GStreamer, even though GStreamer itself is heavily threaded.
        """
        self.bus = self.pipeline.get_bus()
        #After calling this statement, the bus will emit the "message" signal for each message posted on the bus.
        self.bus.add_signal_watch()
        self.bus_id = self.bus.connect('message::application', self.application_message)
        self.pipeline.set_state(gst.STATE_PLAYING)
        self.started = True

    def pulse_index_from_name(self, name):
        output = commands.getstatusoutput("pacmd list-sources | grep -B 1 'name: <" + name + ">' | grep -o -P '(?<=index: )[0-9]*'")

        if len(output) == 2:
            return output[1]
        else:
            raise Exception("Error. pulse index doesn't exist for name: " + name)

    def stop_recognizer(self):
        if self.started:
            self.pipeline.set_state(gst.STATE_NULL)
            self.pipeline.remove(self.asr)
            self.bus.disconnect(self.bus_id)
            self.started = False

    def shutdown(self):
        """ Delete any remaining parameters so they don't affect next launch """
        for param in [self._device_name_param, self._lm_param, self._dic_param]:
            if rospy.has_param(param):
                rospy.delete_param(param)

        """ Shutdown the GTK thread. """
        gtk.main_quit()

    def start(self, req):
        self.start_recognizer()
        rospy.loginfo("recognizer started")
        return EmptyResponse()

    def stop(self, req):
        self.stop_recognizer()
        rospy.loginfo("recognizer stopped")
        return EmptyResponse()

    def asr_partial_result(self, asr, text, uttid):
        """ Forward partial result signals on the bus to the main thread. """
        struct = gst.Structure('partial_result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def asr_result(self, asr, text, uttid):
        """ Forward result signals on the bus to the main thread. """
        struct = gst.Structure('result')
        struct.set_value('hyp', text)
        struct.set_value('uttid', uttid)
        asr.post_message(gst.message_new_application(asr, struct))

    def application_message(self, bus, msg):
        """ Receive application messages from the bus. """
        msgtype = msg.structure.get_name()
        if msgtype == 'partial_result':
            self.partial_result(msg.structure['hyp'], msg.structure['uttid'])
        if msgtype == 'result':
            self.final_result(msg.structure['hyp'], msg.structure['uttid'])

    def partial_result(self, hyp, uttid):
        """ Delete any previous selection, insert text and select it. """
        rospy.logdebug("Partial: " + hyp)

    def final_result(self, hyp, uttid):
        """ Insert the final result. """
        msg = String()
        msg.data = str(hyp.lower())
        rospy.loginfo(msg.data)
        self.pub.publish(msg)

if __name__ == "__main__":
    start = recognizer()


#!/usr/bin/python

import os

import rospkg

import rospy

import pyaudio

from std_msgs.msg import String

class AudioMessage(object):
    def __init__(self):
        # Start node
        self.pub_ = rospy.Publisher("sphinx_msg", String, queue_size=1)
        rospy.init_node("audio_control")
        rospy.on_shutdown(self.shutdown)

        self.transfer_audio_msg()


    def transfer_audio_msg(self):
        # Params
        self._input = "~input"

        rospack = rospkg.RosPack()

        self.location = rospack.get_path('pocketsphinx') + '/demo/'

        if rospy.has_param(self._input):
            if rospy.get_param(self._input) != ":default":
                stream = open(os.path.join(self.location + rospy.get_param(self._input)), 'rb')
            else:
                stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                                rate=16000, input=True, frames_per_buffer=1024)
                stream.start_stream()

        while not rospy.is_shutdown():
            buf = stream.read(1024)
            if buf:
                self.pub_.publish(buf)
            else:
                break
        self.pub_.publish("ended")

    def shutdown(self):
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop ASRControl")
        rospy.sleep(1)

if __name__ == "__main__":
    start = AudioMessage()
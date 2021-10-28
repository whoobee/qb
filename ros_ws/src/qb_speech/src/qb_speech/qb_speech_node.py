#!/usr/bin/env python3

from __future__ import print_function
import rospy
import std_msgs.msg
from std_msgs.msg import Float64, Int32, String
#from move_base.msg import *
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
import diagnostic_updater, diagnostic_msgs.msg

import time
import _thread

import traceback
import queue

import struct
import pyaudio
import pvporcupine
import azure.cognitiveservices.speech as speechsdk

#ROS logger class
class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #

#ROS parameter parse
def get_param(name, default):
    val = rospy.get_param(name, default)
    rospy.loginfo('  %s: %s', name, str(val))
    return val

#qB class
class QbSpeech(object):  
    #execution rate
    main_rate = 10
    #ros pub freq
    qb_speech_calc_hz = 2
    #state var
    qb_speech_msg = "test"

    #init
    def __init__(self):
        #read params
        self.qb_speech_cmd_topic = get_param('~qb_speech_cmd_topic', "/qb_speech_cmd")
        #setup shutdown hook
        rospy.on_shutdown(self.terminate)
        #setup pub and sub
        self.qb_speech_publisher  = rospy.Publisher(self.qb_speech_cmd_topic, String, queue_size=2)

        self.pcphandle = pvporcupine.create(keywords=['computer', 'jarvis'])
        self.pa = pyaudio.PyAudio()
        self.audio_stream = self.pa.open(
            rate=self.pcphandle.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.pcphandle.frame_length)

        self.speech_config = speechsdk.SpeechConfig(subscription="45383dcf0feb40779965f91bdbd3f904", region="westeurope")
        self.speech_recognizer = speechsdk.SpeechRecognizer(speech_config=self.speech_config)

    #main loop handler
    def main_loop(self):
        # Main control, handle startup and error handling
        # while a ROS timer will handle the high-rate (~50Hz) comms + odometry calcs
        self.main_rate = rospy.Rate(10) # hz
        # Start timer to run high-rate comms
        self.fast_timer = rospy.Timer(rospy.Duration(1/float(self.qb_speech_calc_hz)), self.fast_timer)
    
    #timer setup for main loop cyclic exec
    def fast_timer(self, timer_event):
        #calculate timestamp and publish
        time_now = rospy.Time.now()
        self.pub_state(time_now)
    
    #shutdown hook
    def terminate(self):
        self.qb_speech_msg = "exit"
        self.pcphandle.delete()

    #publisher func
    def pub_state(self, time_now):
        self.qb_speech_publisher.publish(self.qb_speech_msg)

    def get_next_audio_frame(self):
        pass

    def from_mic(self):
        while (True):
            pcm = self.audio_stream.read(self.pcphandle.frame_length)
            pcm = struct.unpack_from("h" * self.pcphandle.frame_length, pcm)

            keyword_index = self.pcphandle.process(pcm)

            if keyword_index >= 0:
                print("Wakeword detected. Listening...")
                result = self.speech_recognizer.recognize_once_async().get()
                print(result.text)
                self.qb_speech_msg = result.text
                time.sleep(1)

#main thread
def start_manager():
    #init ROS node
    rospy.init_node('qb_speech_mngr')
    #init and start the qB class
    qb_speech = QbSpeech()
    #qb_speech.main_loop()
    qb_speech.from_mic()
    # start speech to text thread
    """try:
        _thread.start_new_thread(qb_speech.from_mic())
    except:
        print("Error: unable to start thread")"""
    #shutdown listener
    while not rospy.is_shutdown():
        qb_speech.main_rate.sleep()
    
#main thread entry point
if __name__ == '__main__':
    try:
        start_manager()
    except rospy.ROSInterruptException:
        pass


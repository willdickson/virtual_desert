#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import time
import copy
import yaml
import json
import threading
import numpy as np
import rospy
import angle_utils
import lowpass_filter
import std_msgs.msg

from ledpanels import display_ctrl
from basic_led_strip_proxy import BasicLedStripProxy

from rolling_circular_mean import RollingCircularMean
from trial_magno_arena import TrialMagnoArena, DummyTrial

from magnotether.msg import MsgAngleData

from virtual_desert.msg import TrialDataMagnoArena
from virtual_desert.msg import VirtualDesertDataMagnoArena


#class VirtualDesert (object):
class MagnoArena (object):

    Default_Param_File = 'virtual_desert_param.yaml'

    def __init__(self):

        self.current_trial = DummyTrial() 
        self.current_trial_index = None 

        #rospy.init_node('virtual_desert')
        rospy.init_node('magno_arena')
        
        self.get_param()
        self.rate = rospy.Rate(self.param['update_rate'])

        self.lock = threading.Lock()
        self.start_time = rospy.get_time()

        self.angle_lowpass_filter = lowpass_filter.LowpassFilter(self.param['angle_lowpass_fcut'])

        self.devices = {}
        self.devices['panels_controller'] = display_ctrl.LedController()
        self.devices['sunled_proxy'] = BasicLedStripProxy()

        self.initialize_panels_controller()
        self.rolling_circ_mean = RollingCircularMean(self.param['rolling_mean_size'])

        self.rolling_circ_mean = RollingCircularMean(self.param['rolling_mean_size'])
        self.angle_data_sub = rospy.Subscriber('/angle_data', MsgAngleData,self.on_angle_data_callback)
        
        self.data_pub = rospy.Publisher('/magno_arena_data', VirtualDesertDataMagnoArena, queue_size=10)
        self.param_pub = rospy.Publisher('/magno_arena_param', std_msgs.msg.String, queue_size=10)

    def initialize_panels_controller(self):
        # Wait until we have a subscriber connected or else message may be thrown away
        # It would be better if the panels controller used a service.
        while self.devices['panels_controller'].pub.get_num_connections() < 1:
            # May want to put a try counter on this and error out after some number of attempts
            rospy.sleep(0.1)
        self.devices['panels_controller'].set_config_id(self.param['panels_config_id'])
        self.devices['panels_controller'].stop()      

    def shutdown_panels(self):
        self.devices['panels_controller'].stop()
        self.devices['panels_controller'].all_off()

    def get_param(self):
        self.param = rospy.get_param('/virtual_desert', None) 
        if self.param is None:
            param_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),self.Default_Param_File)
            with open(param_file_path,'r') as f:
                self.param = yaml.load(f)

    @property
    def angle_dt(self):
        return 1.0/self.param['angle_framerate']

    @property
    def elapsed_time(self):
        return rospy.get_time() - self.start_time

    @property
    def mean_angle(self):
        with self.lock: 
            mean_angle = self.rolling_circ_mean.value()
        return mean_angle
    
    @property
    def angle(self):
        with self.lock:
            angle = self.angle_lowpass_filter.value
        return angle
        
    def on_angle_data_callback(self,data):
        with self.lock:
            self.rolling_circ_mean.insert_data(data.angle)

    def move_to_next_trial(self): 
        if self.current_trial_index is None:
            self.current_trial_index = 0
            rospy.logwarn(self.current_trial_index)
        else:
            del self.current_trial
            self.current_trial_index += 1
            rospy.logwarn(self.current_trial_index)
        trial_param = self.get_trial_params(self.current_trial_index)
        self.current_trial = TrialMagnoArena(
                self.elapsed_time, 
                self.mean_angle, 
                trial_param, 
                self.devices, 
                self.current_trial_index
                )

    def get_trial_params(self,index):
        """ 
        Get parameters for trial set flow parameters to default and override
        where specified in trial 
        """
        trial_param = copy.deepcopy(self.param['trials'][index])
        #param_name_list = ['flow', 'panels', 'sunled', 'autostep']
        param_name_list = ['panels', 'sunled']
        for param_name in param_name_list:
            try:
                param = copy.deepcopy(self.param['{}_default'.format(param_name)])
            except KeyError:
                param = {}
            if param is None:
                param = {}
            param.update(trial_param[param_name])
            trial_param[param_name] = param
        return trial_param

    def run(self):

        # Set to true to make sure parameters are in bag file
        if self.param['wait_for_param_sub']: 
            while self.param_pub.get_num_connections() < 1:
                rospy.sleep(0.1)
        self.param_pub.publish(json.dumps(self.param))

        while not rospy.is_shutdown(): 
            elapsed_time = self.elapsed_time
            if elapsed_time > self.param['startup_delay']:
                if self.current_trial.is_done(elapsed_time):
                    try:
                        self.move_to_next_trial()
                    except IndexError:
                        break  # Done with trials -> exit loop
                trial_msg = self.current_trial.update(elapsed_time, self.angle)
                #print('t: {:0.2f}'.format(self.elapsed_time))
            else:
                print('pretrial delay, t={:0.2f}'.format(elapsed_time))
                #trial_msg = TrialDataMagnoArena()

            msg = VirtualDesertDataMagnoArena()
            msg.header.stamp = rospy.Time.now()
            msg.angle = self.angle
            msg.elapsed_time = elapsed_time
            msg.current_trial_index = self.current_trial_index
            msg.trial_data = trial_msg
            self.data_pub.publish(msg)
            self.rate.sleep()

        self.shutdown_panels()


# ---------------------------------------------------------------------------------------

if __name__ == '__main__':

    node = MagnoArena()
    node.run()



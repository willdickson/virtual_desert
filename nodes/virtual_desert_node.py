#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import time
import copy
import yaml
import threading
import numpy as np
import rospy

from ledpanels import display_ctrl
from alicat_ros_proxy import  AlicatProxy

from rolling_circular_mean import RollingCircularMean
from trial import Trial, DummyTrial

from magnotether.msg import MsgAngleData


class VirtualDesert(object):

    Default_Param_File = 'virtual_desert_param.yaml'

    def __init__(self):

        self.current_trial = DummyTrial() 
        self.current_trial_index = None 

        rospy.init_node('virtual_desert')
        self.get_param()
        self.rate = rospy.Rate(50.0)

        self.lock = threading.Lock()
        self.start_time = rospy.get_time()

        self.devices = {}
        self.devices['panels_controller'] = display_ctrl.LedControler()
        self.devices['panels_controller'].set_config_id(self.param['panels_config_id'])
        self.devices['alicat_proxy'] = AlicatProxy()

        self.rolling_circ_mean = RollingCircularMean(self.param['rolling_mean_size'])
        self.angle_data_sub = rospy.Subscriber('/angle_data', MsgAngleData,self.on_angle_data_callback) 

    def get_param(self):
        self.param = rospy.get_param('/virtual_desert/param', None) 
        if self.param is None:
            param_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),self.Default_Param_File)
            with open(param_file_path,'r') as f:
                self.param = yaml.load(f)

    @property
    def elapsed_time(self):
        return rospy.get_time() - self.start_time

    @property
    def mean_angle(self):
        with self.lock: 
            mean_angle = self.rolling_circ_mean.value()
        return mean_angle

    def on_angle_data_callback(self,data):
        with self.lock:
            self.rolling_circ_mean.insert_data(data.angle)

    def move_to_next_trial(self): 
        if self.current_trial_index is None:
            self.current_trial_index = 0
        else:
            del self.current_trial
            self.current_trial_index += 1
        trial_param = self.get_trial_params(self.current_trial_index)
        self.current_trial = Trial(self.elapsed_time, self.mean_angle, trial_param, self.devices)

    def get_trial_params(self,index):
        """ 
        Get parameters for trial set flow parameters to default and override
        where specified in trial 
        """
        trial_param = copy.deepcopy(self.param['trials'][index])
        flow_param = copy.deepcopy(self.param['flow_default'])
        flow_param.update(trial_param['flow'])
        trial_param['flow'] = flow_param
        return trial_param

    def run(self):
        while not rospy.is_shutdown(): 
            if self.elapsed_time > self.param['startup_delay']:
                if self.current_trial.is_done(self.elapsed_time):
                    try:
                        self.move_to_next_trial()
                    except IndexError:
                        break  # Done with trials -> exit loop
                self.current_trial.update(self.elapsed_time, self.mean_angle)
            else:
                print('pretrial delay, t={:0.2f}'.format(self.elapsed_time))
            self.rate.sleep()


# ---------------------------------------------------------------------------------------

if __name__ == '__main__':

    node = VirtualDesert()
    node.run()



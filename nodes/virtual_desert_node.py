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
from rolling_circular_mean import RollingCircularMean
from trial import Trial

from magnotether.msg import MsgAngleData


class VirtualDesert(object):

    Default_Param_File = 'virtual_desert_param.yaml'

    def __init__(self):

        self.current_trial_index = None 

        rospy.init_node('virtual_desert')
        self.get_param()
        self.rate = rospy.Rate(50.0)

        self.lock = threading.Lock()
        self.start_time = rospy.get_time()

        self.devices = {}
        self.devices['panels_controller'] = display_ctrl.LedControler()
        self.devices['panels_controller'].set_config_id(self.param['panels_config_id'])

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

    def on_angle_data_callback(self,data):
        with self.lock:
            self.rolling_circ_mean.insert_data(data.angle)

    def get_next_trial(self): 
        if self.current_trial_index is None:
            self.current_trial_index = 0
        else:
            self.current_trial_index += 1
        try:
            trial_params = copy.deepcopy(self.param['trials'][self.current_trial_index])
            next_trial = Trial(self.elapsed_time, trial_params, self.devices)
        except IndexError:
            next_trial = None
        return next_trial

    def run(self):

        trial = None

        while not rospy.is_shutdown(): 

            if self.elapsed_time > self.param['startup_delay']:
                
                if trial is None or trial.is_done(self.elapsed_time):
                    trial = self.get_next_trial()
                if trial is None:
                    break
                print('trial = {}'.format(trial.name))

                with self.lock:
                    mean_angle = self.rolling_circ_mean.value()

                trial.update(self.elapsed_time,mean_angle)

            else:
                print('pretrial delay, t={:0.2f}'.format(self.elapsed_time))

            self.rate.sleep()


# ---------------------------------------------------------------------------------------

if __name__ == '__main__':

    node = VirtualDesert()
    node.run()



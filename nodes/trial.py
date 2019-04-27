from __future__ import print_function
import rospy

class Trial(object):

    def __init__(self, start_time, trial_param, devices):
        self.start_time = start_time 
        self.param = trial_param
        self.devices = devices

    @property
    def name(self):
        return self.param['name']

    def elapsed_time(self,t):
        return t-self.start_time 

    def is_done(self,t):
        return self.elapsed_time(t) >= self.param['duration']

    def update(self,t,angle):
        print('t: {:0.2f}, et: {:0.2f}, ang: {}'.format(t, self.elapsed_time(t),angle))





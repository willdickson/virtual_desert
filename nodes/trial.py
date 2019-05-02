from __future__ import print_function
import rospy

from flow_action import FlowAction
from panels_action import PanelsAction


class Trial(object):

    def __init__(self, start_time, init_angle, trial_param, devices):
        self.start_time = start_time 
        self.init_angle = init_angle
        self.param = trial_param
        self.devices = devices

        self.flow_action = FlowAction(self.devices['alicat_proxy'], self.param['flow'])
        self.panels_action = PanelsAction(self.init_angle,self.devices['panels_controller'], self.param['panels'])
        self.action_list = [self.flow_action, self.panels_action]

    def __del__(self):
        self.device_shutdown()

    def device_shutdown(self):
        for action in self.action_list:
            if not action.is_stopped:
                action.stop()

    @property
    def name(self):
        return self.param['name']

    def elapsed_time(self,t):
        return t-self.start_time 

    def is_done(self,t):
        return self.elapsed_time(t) >= self.param['duration']

    def update(self,t,angle):
        for action in self.action_list:
            action.update(self.elapsed_time(t))
        #print('{}: et: {:0.2f}, ang: {}'.format(self.name, self.elapsed_time(t),angle))


class DummyTrial(object):

    def __init__(self):
        pass

    def is_done(self,t):
        return True








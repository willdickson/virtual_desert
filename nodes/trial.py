from __future__ import print_function
import rospy
from flow_action import FlowAction


class Trial(object):

    def __init__(self, start_time, init_angle, trial_param, devices):
        self.start_time = start_time 
        self.init_angle = init_angle
        self.param = trial_param
        self.devices = devices

        self.flow_action = FlowAction(self.devices['alicat_proxy'], self.param['flow'])
        self.action_list = [self.flow_action]

    def __del__(self):
        self.device_shutdown()

    def device_shutdown(self):
        self.flow_action.set_flow(0.0)


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

        print('{}: et: {:0.2f}, ang: {}'.format(self.name, self.elapsed_time(t),angle))
        pass


class DummyTrial(object):

    def __init__(self):
        pass

    def is_done(self,t):
        return True








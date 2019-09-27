from __future__ import print_function
import rospy

from flow_action import FlowAction
from panels_action import PanelsAction
from autostep_action import AutostepAction
from sunled_action import SunledAction

from virtual_desert.msg import TrialData
from virtual_desert.msg import ActionData


class Trial(object):

    def __init__(self, start_time, init_angle, trial_param, devices):
        self.start_time = start_time 
        self.init_angle = init_angle
        self.param = trial_param
        self.devices = devices

        self.autostep_action = AutostepAction(
                self.devices['autostep_tracking_data_pub'], 
                self.devices['autostep_proxy'], 
                self.param['autostep']
                )

        self.flow_action = FlowAction(
                self.devices['alicat_proxy'], 
                self.param['flow']
                )

        self.panels_action = PanelsAction(
                self.init_angle,
                self.devices['panels_controller'], 
                self.param['panels']
                )

        #self.sunled_action = SunledAction(
        #        self.init_angle,
        #        self.devices['sunled_proxy'], 
        #        self.param['sunled']
        #        )
        #self.action_list = [self.autostep_action, self.panels_action, self.flow_action, self.sunled_action]

        self.action_list = [self.autostep_action, self.panels_action, self.flow_action]

    def __del__(self):
        self.device_shutdown()

    def device_shutdown(self):
        for action in reversed(self.action_list):
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
        msg = TrialData()
        msg.name = self.name
        msg.elapsed_time = self.elapsed_time(t)
        msg.init_angle = self.init_angle
        msg.autostep_action_data = self.autostep_action.update(self.elapsed_time(t),angle)
        msg.panels_action_data = self.panels_action.update(self.elapsed_time(t),angle)
        msg.flow_action_data = self.flow_action.update(self.elapsed_time(t),angle)
        return msg


class DummyTrial(object):

    def __init__(self):
        pass

    def is_done(self,t):
        return True








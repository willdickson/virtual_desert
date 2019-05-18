"""
base_action.py

Base class for trial actions e.g. flow_action, panels_action, etc.
"""

from virtual_desert.msg import ActionData

class BaseAction(object):

    def __init__(self,device,param):
        self.is_started = False
        self.is_stopped = False
        self.device = device
        self.param = param

    @property
    def is_running(self):
        return self.is_started and not self.is_stopped

    def start(self):
        self.is_started = True

    def stop(self):
        self.is_stopped = True

    def update(self,t,angle):
        if not self.is_started:
            try:
                start_time = self.param['start_time']
            except KeyError:
                start_time = 0.0
            if t >= start_time: 
                self.start()
        if not self.is_stopped: 
            try:
                stop_time = self.param['stop_time']
            except KeyError:
                stop_time = None
            if stop_time is not None and t >= stop_time:
                self.stop()

        msg = ActionData()
        msg.is_started = self.is_started
        msg.is_stopped = self.is_stopped
        msg.is_running = self.is_running
        return msg

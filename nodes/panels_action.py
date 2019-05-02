from __future__ import print_function
from base_action import BaseAction
import numpy as np

class PanelsAction(BaseAction):

    def __init__(self,init_angle,device,param):
        print('panels action __init__')
        super(PanelsAction,self).__init__(device,param)
        self.init_angle = init_angle

    def stop(self):
        print('panels action stop')
        if not self.is_stopped:
            self.panels_off()
            self.is_stopped = True

    def start(self):
        print('panels action start')
        if not self.is_started:
            pattern_id = self.get_pattern_id()
            print('pattern_id: ', pattern_id)
            self.show_pattern(pattern_id)
            self.is_started = True

    def show_pattern(self,pattern_id):
        if pattern_id == 'off':
            self.panels_off()
        else:
            self.device.stop()
            self.device.set_pattern_id(pattern_id)      
            self.device.send_gain_bias(
                    gain_x = self.param['gain_x'], 
                    bias_x = self.param['bias_x'],
                    gain_y = self.param['gain_x'],
                    bias_y = self.param['bias_x']
                    )
            self.device.start()

    def panels_off(self):
        self.device.stop()
        self.device.all_off()

    def get_pattern_id(self):
        if self.param['mode'] == 'fixed_pattern':
            pattern_id = self.param['pattern_id']
        else:
            angle_pair_list = [angle_pair for angle_pair,pair_id in self.param['pattern_table']]
            pair_id_list  = [pair_id for angle,pair_id in self.param['pattern_table']]
            pattern_id = 'off'
            for angle_pair, pair_id in zip(angle_pair_list, pair_id_list):
                lower_angle, upper_angle = angle_pair
                if lower_angle <= self.init_angle and self.init_angle <= upper_angle:
                    pattern_id = pair_id
                    break
        return pattern_id


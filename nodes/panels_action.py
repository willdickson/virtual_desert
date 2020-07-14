from __future__ import print_function
from base_action import BaseAction
import numpy as np
import rospy

class PanelsAction(BaseAction):

    index_to_patter_id = {}

    def __init__(self,init_angle,device,param,trial_index): #fponce edit 
        print('panels action __init__')
        super(PanelsAction,self).__init__(device,param)
        self.init_angle = init_angle
        #fponce edit
        self.pattern_id = None
        self.last_update_t = None 
        self.trial_index = trial_index

    def stop(self):
        print('panels action stop')
        if not self.is_stopped:
            if not ('nostop' in self.param):
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
        #fponce edit
        if self.param['mode'] == 'inherit_from_last':
            rospy.logwarn('pass show pattern command')
            return
       ############
        if pattern_id == 'off':
            self.panels_off() 
                     
        else:
            print('show pattern', pattern_id)
            self.device.set_pattern_id(pattern_id)      
            self.device.stop()
            self.device.send_gain_bias(
                    gain_x = self.param['gain_x'], 
                    bias_x = self.param['bias_x'],
                    gain_y = self.param['gain_y'],
                    bias_y = self.param['bias_y']
                    )
            self.device.start()

    def panels_off(self):
        self.device.stop()
        self.device.all_off()
        

    def get_pattern_id(self):
        if self.param['mode'] == 'fixed_pattern':
            pattern_id = self.param['pattern_id']
            #fponce edit##
            self.index_to_patter_id[self.trial_index] = pattern_id
        #fponce edit
        elif self.param['mode'] == 'inherit_from_last':
            inherit_index = self.trial_index-1
            pattern_id = self.index_to_patter_id[inherit_index]
            self.index_to_patter_id[self.trial_index] = pattern_id

        elif self.param['mode'] == 'inherit':
            inherit_index = self.param['inherit_from']
            pattern_id = self.index_to_patter_id[inherit_index]
            self.index_to_patter_id[self.trial_index] = pattern_id
            
        elif self.param['mode'] == 'inherit_n_set_by_table':
            inherit_index = self.param['inherit_from']
            
            self.prev_pattern = self.index_to_patter_id[inherit_index]
            self.position = self.get_pattern_from_patterntable(self.prev_pattern)
            pattern_id =  self.position
            self.index_to_patter_id[self.trial_index] = self.position

        #####
        else:
            angle_pair_list = [angle_pair for angle_pair,pair_id in self.param['pattern_table']]
            pair_id_list  = [pair_id for angle,pair_id in self.param['pattern_table']]
            pattern_id = 'off'
            for angle_pair, pair_id in zip(angle_pair_list, pair_id_list):
                lower_angle, upper_angle = angle_pair
                if lower_angle <= self.init_angle and self.init_angle <= upper_angle:
                    pattern_id = pair_id
                    #fponce edit
                    self.index_to_patter_id[self.trial_index] = pattern_id
                    break
        return pattern_id
        
######################
    #fponce edit# get a pattern based on what pattern was shown in the past in a different trial
    def get_pattern_from_patterntable(self,prev_position): 
        prev_pattern_list = [prev_pattern for prev_pattern,pattern_index in self.param['pattern_to_pattern_table']]
        pattern_index_list  = [pattern_index for old_pattern,pattern_index in self.param['pattern_to_pattern_table']]
        position = None
        for prev_pattern, pattern_index in zip(prev_pattern_list, pattern_index_list):
            if prev_pattern == prev_position:
                position = pattern_index
        return position


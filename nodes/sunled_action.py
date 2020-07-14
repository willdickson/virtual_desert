import math
import rospy
import numpy as np
import random
from base_action import BaseAction

class SunledAction(BaseAction):

    index_to_led_position = {}

    def __init__(self,init_angle,device,param,trial_index):
        print('sunled action __init__')
        super(SunledAction,self).__init__(device,param)
        self.init_angle = init_angle

        self.position = 0
        self.last_update_t = None 
        self.trial_index = trial_index

    def update(self,t,angle):
        rval_msg = super(SunledAction,self).update(t,angle)
        if self.param['mode'] == 'inherit_from':
            pass
        if self.last_update_t is None:
            self.last_update_t = t
        dt = t - self.last_update_t
        if dt > self.param['update_period']:
            if self.param['mode'] == 'fixed_rate':
                self.position = dt*self.param['rate'] + self.position
                self.position = np.mod(self.position, self.param['number_of_leds'])
                self.device.set_led(int(self.position),self.param['rgb_value'])
            self.last_update_t = t
        return rval_msg

    def start(self):
        if not self.is_started:
            #rospy.logwarn(self.param['mode'])
            if self.param['mode'] in ('fixed_position', 'fixed_rate'):
                if self.param['position'] == 'inherit':
                    inherit_index = self.param['inherit_from']
                    self.position = self.index_to_led_position[inherit_index]
                else:
                    self.position = self.param['position']
                self.device.set_led(int(self.position),self.param['rgb_value'])
                self.index_to_led_position[self.trial_index] = self.position
            elif self.param['mode'] == 'set_by_angle':
                self.position = self.get_position_from_table(self.init_angle)
                self.index_to_led_position[self.trial_index] = self.position 
                if self.position is not None:
                    self.device.set_led(int(self.position),self.param['rgb_value'])
            
            #fponce edit 23/01/2020
            elif self.param['mode'] == 'all_ON':
                if self.position is not None:
                    self.device.set_all(self.param['rgb_value'])
                    self.index_to_led_position[self.trial_index] = -1
                                
            #fponce edit   
            elif self.param['mode'] == 'inherit_n_set_by_table':
                inherit_index = self.param['inherit_from']
                self.prev_position = self.index_to_led_position[inherit_index]
                self.position = self.get_position_from_ledtable(self.prev_position)
                self.index_to_led_position[self.trial_index] = self.position
                if self.position is not None:
                    self.device.set_led(int(self.position),self.param['rgb_value'])
                  
            elif self.param['mode'] == 'inherit_from_last':
                inherit_index = self.trial_index-1
                self.position = self.index_to_led_position[inherit_index]
                self.index_to_led_position[self.trial_index] = self.position 
                  
            elif self.param['mode'] == 'random_from_list':
                self.position = self.get_random_from_list(self.param['sunled_position_list'])
                self.index_to_led_position[self.trial_index] = self.position
                if self.position is not None:
                    self.device.set_led(int(self.position),self.param['rgb_value'])
            ##########
            
            else:
                raise ValueError, 'unknown mode'
            #rospy.logwarn(self.position)
            self.is_started = True

#    def stop(self):
#        if not self.is_stopped:
#            self.device.set_led(-1,(0,0,0))
#            self.is_stopped = True

    def get_position_from_table(self,angle): 
        angle_pair_list = [angle_pair for angle_pair,led_index in self.param['sunled_table']]
        led_index_list  = [led_index for angle,led_index in self.param['sunled_table']]
        position = None
        for angle_pair, led_index in zip(angle_pair_list, led_index_list):
            lower_angle, upper_angle = angle_pair
            if lower_angle <= self.init_angle and self.init_angle <= upper_angle:
                position = led_index
                break
        return position 

######################
    #fponce edit
    def get_position_from_ledtable(self,prev_position): 
        prev_led_list = [prev_led for prev_led,led_index in self.param['led_to_led_table']]
        led_index_list  = [led_index for old_led,led_index in self.param['led_to_led_table']]
        position = None
        for prev_led, led_index in zip(prev_led_list, led_index_list):
            if prev_led == prev_position:
                position = led_index
        return position
        
    def get_random_from_list(self, position_list):
        position = random.choice(position_list)
        return position
    


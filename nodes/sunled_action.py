import math
import rospy
from base_action import BaseAction

class SunledAction(BaseAction):

    def __init__(self,init_angle,device,param):
        print('sunled action __init__')
        super(SunledAction,self).__init__(device,param)
        self.init_angle = init_angle

        self.position = 0
        self.last_update_t = None

    def update(self,t,angle):
        rval_msg = super(SunledAction,self).update(t,angle)
        if self.param['mode'] == 'fixed_rate':
            if self.last_update_t is not None:
                dt = t - self.last_update_t
                self.position = dt*self.param['rate'] + self.position
                # Too slow for 50Hz - call less often
                self.set_led_with_dither(self.position)

            self.last_update_t = t
        return rval_msg

    def start(self):
        if not self.is_started:

            if self.param['mode'] == 'fixed_position':
                if type(self.param['position']) == list:
                    position_list = self.param['position']
                else:
                    position_list = [self.param['position']]
                for pos in position_list:
                    self.set_led_with_dither(self.position)

            elif self.param['mode'] == 'fixed_rate':
                self.position = self.param['position']
                self.set_led_with_dither(self.position)

            elif self.param['mode'] == 'set_by_angle':
                pass

            elif self.param['mode'] == 'inherit':
                pass

            else:
                raise ValueError, 'unknown mode'
            self.is_started = True

    def stop(self):
        if not self.is_stopped:
            self.device.set_led(-1,(0,0,0))
            self.is_stopped = True

    def set_led_with_dither(self,position):
        self.device.set_led(-1,(0,0,0)) # turn all leds off

        lower_led = int(math.floor(position))
        upper_led = int(math.floor(position))

        upper_scale = position - math.floor(position)
        lower_scale = 1.0 - upper_frac

        upper_rgb = [int(upper_scale*x) for x in self.param['rgb_value']]
        lower_rgb = [int(lower_scale*x) for x in self.param['rgb_value']]

        self.device.set_led(lower_led, lower_rgb)
        self.device.set_led(upper_led, upper_rgb)
        



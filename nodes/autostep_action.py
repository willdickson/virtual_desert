from base_action import BaseAction


class AutostepAction(BaseAction):

    def __init__(self, tracking_data_pub, device, param):
        super(AutostepAction,self).__init__(device,param)
        self.tracking_data_pub = tracking_data_pub

    def update(self,t,angle):
        super(AutostepAction,self).update(t,angle)

    def start(self):
        if not self.is_started:
            print('Autostep stop action')
            if self.param['mode'] == 'fixed_angle':
                self.device.set_move_mode('jog')
                self.device.move_to(self.param['angle'])
                self.device.busy_wait()
            elif self.param['mode'] == 'fixed_rate':
                self.device.set_move_mode('max')
                self.device.run(self.param['rate'])
            elif self.param['mode'] == 'closed_loop':
                self.device.enable_tracking_mode()
            else:
                raise RuntimeError, 'unknown mode {}'.format(self.param['mode'])
            self.is_started = True

    def stop(self):
        if not self.is_stopped:
            print('Autostep stop action')
            if self.param['mode'] == 'closed_loop':
                self.device.disable_tracking_mode()
            self.device.set_move_mode('jog')
            self.device.soft_stop()
            self.device.busy_wait()
            self.device.move_to(0.0)
            self.device.busy_wait()
            self.is_stopped = True




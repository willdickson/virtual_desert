from base_action import BaseAction

class FlowAction(BaseAction):

    def __init__(self, device, param):
        super(FlowAction,self).__init__(device,param)

    def stop(self):
        if not self.is_stopped:
           self.set_flow(0.0)
           self.is_stopped = True

    def start(self):
        if not self.is_started:
            self.set_flow(self.param['set_point'])
            self.is_started = True

    def set_flow(self,set_point):
        flowrate_dict = {}
        push_scale = self.param['push_scale']
        push_addresses = self.param['push_addresses']
        for addr in push_addresses:
            flowrate_dict[addr] = set_point*push_scale/len(push_addresses)
        pull_scale = self.param['pull_scale']
        pull_addresses = self.param['pull_addresses']
        for addr in pull_addresses:
            flowrate_dict[addr] = set_point*pull_scale/len(pull_addresses)
        rsp = self.device.set_flow_rate(flowrate_dict)




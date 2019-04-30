import threading

class FlowAction(object):

    def __init__(self, device, param):
        self.is_started = False
        self.device = device
        self.param = param

    def update(self,t):
        if not self.is_started:
            try:
                start_time = self.param['start_time']
            except KeyError:
                start_time = 0.0
            if t >= start_time: 
                self.set_flow(self.param['set_point'])
                self.is_started = True
        else:
            try:
                stop_time = self.param['stop_time']
            except KeyError:
                stop_time = None
            if stop_time is not None and t >= stop_time:
                self.set_flow(0.0)

    def set_flow(self,set_point):
        print('set flow {}'.format(set_point))
        flowrate_dict = {}
        push_scale = self.param['push_scale']
        push_addresses = self.param['push_addresses']
        for addr in push_addresses:
            flowrate_dict[addr] = set_point*push_scale/len(push_addresses)
        pull_scale = self.param['pull_scale']
        pull_addresses = self.param['pull_addresses']
        for addr in pull_addresses:
            flowrate_dict[addr] = set_point*pull_scale/len(pull_addresses)

        # Set flow rate using daemon thread to avoid delay
        proxy_thread = threading.Thread(target=self.device.set_flow_rate, args=(flowrate_dict,))
        proxy_thread.daemon = True
        proxy_thread.start()



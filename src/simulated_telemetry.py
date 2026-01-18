import json
import numpy as np

def interpolate_value(values, timestamps, target_ts):
    return np.interp(target_ts, timestamps, values)


class SimulatedReceiver():
    d = ''
    def __init__(self,path,to):
        self.time_offset = to
        self.telemetry_data_path = path
        
        with open(self.telemetry_data_path) as json_data:
            self.d = json.load(json_data)

    def interpolate(self,t,var):
        return np.interp(t+self.time_offset,self.d['telemetries'][var]['data'][0],self.d['telemetries'][var]['data'][1])


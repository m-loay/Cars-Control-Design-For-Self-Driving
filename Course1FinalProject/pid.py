'''
Created on Aug 15, 2019

@author: modys
'''
import numpy as np
import matplotlib.pyplot as plt
class PID():
    '''
    classdocs
    '''


    def __init__(self,Kp, Ki, Kd):
        '''
        Constructor
        '''
        self._Kp = Kp;
        self._Ki = Ki;
        self._Kd = Kd;
        
        self._max_output = 0.0;
        self._min_output = 0.0;
        self._sample_time = 0.0;
        
        self._error =0.0;
        self._intergal = 0.0;
        self._derivative =0.0;
        self._output = 0.0;
        
    def output_limits(self):
        return self._min_output, self._max_output;
    
    def _clamp(self,value, limits):
        lower, upper = limits
        if value is None:
            return None
        elif upper is not None and value > upper:
            return upper
        elif lower is not None and value < lower:
            return lower
        return value
        
    def update(self, set_point, current_val, sample_time):
        self._sample_time = sample_time;
        last_error        = self._error;
        last_intgeral     = self._intergal;

        #Error Part
        self._error = set_point - current_val;
        
        #Intgeral Part
        self._intergal = last_intgeral + (self._error * self._sample_time) ;
        self._intergal = self._clamp(self._intergal, self.output_limits)
        
        #derivative part
        self._derivative = (self._error - last_error) / self._sample_time;
        
        #Final Ouput
        self._output = (self._Kp*self._error) + (self._Ki*self._intergal) + (self._Kd*self._derivative)
        self._output = self._clamp(self._output, self.output_limits)
            
        return (self._output);
        
#!/usr/bin/env python3


import sys
from dynamic_reconfigure.parameter_generator_catkin import *
import rospy
from dynamic_reconfigure.encoding import extract_params
from dynamic_reconfigure.server import Server


# define a class so that we don't end up with symbols min, max etc polluting our globals namesapce
class dynamic_config(object):
    def __init__(self, config_description):
        self.config_description = config_description
        self.inf = float('inf')
        self.min = {}
        self.max = {}
        self.defaults = {}
        self.level = {}
        self.type = {}
        self.all_level = 0
        for param in extract_params(config_description):
            self.min[param['name']] = param['min']
            self.max[param['name']] = param['max']
            self.defaults[param['name']] = param['default']
            self.level[param['name']] = param['level']
            self.type[param['name']] = param['type']
            self.all_level = self.all_level | param['level']




# hack:  trick the ParameterGenerator() class from the dynamic_reconfigure module into thinking that the right number of parameters was passed in
_a = sys.argv
sys.argv = ['a','.','.','.','.']
params = ParameterGenerator()
sys.argv = _a
# end of hack

srv = None

def start(callback):
    global params, srv
    config_description = params.group.to_dict()
    config = dynamic_config(config_description)       
    srv = Server(config, callback)


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
gen = ParameterGenerator()
sys.argv = _a
# end of hack


# define the parameters

gen.add("int_param",    int_t,    0, "An Integer parameter", 50,  0, 100)
gen.add("double_param", double_t, 0, "A double parameter",    .5, 0,   1)
gen.add("str_param",    str_t,    0, "A string parameter",  "Hello World")
gen.add("bool_param",   bool_t,   0, "A Boolean parameter",  True)

size_enum = gen.enum([ gen.const("Small",      int_t, 0, "A small constant"),
                       gen.const("Medium",     int_t, 1, "A medium constant"),
                       gen.const("Large",      int_t, 2, "A large constant"),
                       gen.const("ExtraLarge", int_t, 3, "An extra large constant")],
                     "An enum to set size")

gen.add("size", int_t, 0, "A size parameter which is edited via an enum", 1, 0, 3, edit_method=size_enum)

config_description = gen.group.to_dict()

config = dynamic_config(config_description)       


def callback(config, level):
    #rospy.loginfo("""Reconfigure Request: {int_param}""".format(**config))
    print(config)
    return config


if __name__ == "__main__":
    rospy.init_node("dynamic_reconfigure_test", anonymous = False)

    srv = Server(config, callback)
    rospy.spin()



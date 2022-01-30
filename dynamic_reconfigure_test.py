#!/usr/bin/env python3

import rospy
import reconfiguration as r


r.params.add("int_param",    r.int_t,    0, "An Integer parameter", 50,  0, 100)
r.params.add("double_param", r.double_t, 0, "A double parameter",    .5, 0,   1)
r.params.add("str_param",    r.str_t,    0, "A string parameter",  "Hello World")
r.params.add("bool_param",   r.bool_t,   0, "A Boolean parameter",  True)

size_enum = r.params.enum([ r.params.const("Small",      r.int_t, 0, "A small constant"),
                       r.params.const("Medium",     r.int_t, 1, "A medium constant"),
                       r.params.const("Large",      r.int_t, 2, "A large constant"),
                       r.params.const("ExtraLarge", r.int_t, 3, "An extra large constant")],
                     "An enum to set size")

r.params.add("size", r.int_t, 0, "A size parameter which is edited via an enum", 1, 0, 3, edit_method=size_enum)



def config_callback(config, level):
    #rospy.loginfo("""Reconfigure Request: {int_param}""".format(**config))
    print(config)
    return config



if __name__ == "__main__":
    rospy.init_node("dynamic_reconfigure_test_2", anonymous = False)

    r.start(config_callback)

    rospy.spin()



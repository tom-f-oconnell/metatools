#!/usr/bin/env python

import rospy

# i would kind of prefer to kill the roslaunch group / one of the required nodes, rather than
# have this constantly spin, but maybe it is such low overhead that it doesn't matter...

def exactly_one_of(alternatives, all_current_params):
    return sum(map(lambda x: x in all_current_params, alternatives)) == 1


def all_if_any_of(alternatives, all_current_params):
    n_present = sum(map(lambda x: x in all_current_params, alternatives))
    if n_present > 0:
        return n_present == len(alternatives)
    else:
        return True


def err(message):
    error_prefix = 'parameter validator: '
    rospy.logfatal(error_prefix + message)


def get_list(name):
    this_param_namespace = 'parameter_validator/'
    return rospy.get_param(this_param_namespace + name, [])


if __name__ == '__main__':
    delay_s = rosy.get_param('parameter_validator/get_params_after_s', 0)
    rospy.sleep(delay_s)

    # TODO how to normalize parameter names?
    param_names = set(rospy.get_param_names())

    required = get_list('required')
    for p in required:
        if p not in param_names:
            err('parameter ' + p + ' required')

    # should be a list of lists, where each sublist contains a list of parameters
    # of which only one should ever be set at a time
    exactly_one_of_each = get_list('exactly_one_of_each ')
    for l in exactly_one_of_each:
        if not exactly_one_of(l, param_names):
            err('can not have more than one of ' + l + ' set')
    
    # another list of lists, where if any one of the sublist items is set,
    # all the remaining items must also be set
    all_if_any_of_each = get_list('all_if_any_of_each')
    for l in all_if_any_of_each:
        if not all_if_any_of(l, param_names):
            err('must set all of ' + l + ', if any are set')

    # TODO support comparing two parameters
    # and support comparing parameters to constants (min / max)
    # TODO can these be dicts? (specified in yaml?)
    inclusive_upper_bounds = get_list('inclusive_upper_bounds')
    for param, name_or_const_bound in inclusive_upper_bounds:
        # try to catch error when parameters aren't found (to print better error message)?
        value = rospy.get_param(param)
        if type(name_or_const_bound) is str:
            ok = value <= rospy.get_param(name_or_const_bound)
        else:
            ok = value <= name_or_const_bound

        if not ok:
            err(param + ' was greater than inclusive upper bound defined in validator configuration file: ' + name_or_const_bound)

    inclusive_lower_bounds = get_list('inclusive_lower_bounds')
    for param, name_or_const_bound in inclusive_upper_bounds:
        # try to catch error when parameters aren't found (to print better error message)?
        value = rospy.get_param(param)
        if type(name_or_const_bound) is str:
            ok = value >= rospy.get_param(name_or_const_bound)
        else:
            ok = value >= name_or_const_bound

        if not ok:
            err(param + ' was less than inclusive lower bound defined in validator configuration file: ' + name_or_const_bound)

    # TODO err if there are other parameters in this_param_namespace not recognized here
    rospy.spin()

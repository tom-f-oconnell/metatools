#!/usr/bin/env python

import rospy
import rospkg
# i can't figure out how to get package names of running nodes from this
# import rosnode
# this way to import gitpython?
import git
from os.path import split

# required for rospy logging
rospy.init_node('versionpub')

n_src_dirs = 0
for d in rospkg.get_ros_paths():
    # TODO how to find if there are multiple catkin workspaces?
    # probably need to call get_path on RosPack initialized on both, and handle empty output
    if 'src' in split(d):
        source_dir = d
        if n_src_dirs > 0:
            rospy.logwarn('found multiple directories with package sources. only searching first found. this script not currently equipped for this case.')
        n_src_dirs += 1

if source_dir is None:
    rospy.logerr('No source directories found! This package is only currently for storing versions of catkin packages built from source.')

rospack = rospkg.RosPack([source_dir])
rospy.loginfo('versionpub: searching for packages sources in ' + source_dir)

# TODO how to get all package names of running nodes?

# this right way to use "private" params? are these dumped with rosparam?
# TODO what does the tilde do?
package_names = rospy.get_param('~package_names', [])
rospy.logwarn(package_names)

if len(package_names) == 0:
    rospy.logerr('No package names given to versionpub! Pass a list of package names as a private parameter _package_names to the versionpub node, to save the current Git information for those packages.')
else:
    rospy.loginfo('versionpub: checking packages ' + str(package_names))

# TODO also set parameters for the remotes for each repo?

for package_name in package_names:
    # TODO catch error + warn if not found?
    try:
        pkg_path = rospack.get_path(package_name)
    except rospkg.common.ResourceNotFound:
        # TODO how to logerr the stacktrace? traceback module?
        rospy.logfatal(package_name + ' not found in ' + source_dir)
    
    repo = git.Repo(pkg_path, search_parent_directories=True)
    # this a string?
    current_hash = repo.head.object.hexsha
    # will this be in the namespace this node is started in?
    rospy.set_param(package_name + '/commit_hash', current_hash)

    index = repo.index

    # TODO warn if there are any changes / untracked files?

    # TODO list untracked files? current diff approach does not.

    # repr / str?
    # TODO should i always do 0? when will the returned object ever be longer? multiple files?
    # TODO list the equivalent git command
    # TODO alternative to using create_patch?
    # TODO what will this look like if no changes? different from no repo, etc?
    diff = index.diff(None, create_patch=True)
    changes = ''
    for d in diff:
        #rospy.logwarn(d)
        changes += str(d)
    
    """
    if len(diff) == 0:
        changes = ''
    elif len(diff) == 1:
        changes = str(diff[0])
    else:
        rospy.logfatal('gitpython diff had more elements than expected! fix code to handle.')
    """

    rospy.set_param(package_name + '/uncommitted_changes', changes)


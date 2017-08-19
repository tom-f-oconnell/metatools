#!/usr/bin/env python

import rospy
# this way to import gitpython?
import git

repo = git.Repo(search_parent_directories=False)
# this a string?
current_hash = repo.head.object.hexsha
# will this be in the namespace this node is started in?
rospy.set_param('code/commit_hash', current_hash)

index = repo.index

# repr / str?
# TODO should i always do 0? when will the returned object ever be longer?
# TODO list the equivalent git command
patch = str(index.diff(None, create_patch=True)[0])
rospy.set_param('code/uncommitted_changes', patch)

# need to delay at all?


#!/usr/bin/env python

import rospy
import actionlib
import sys

# Import the messages that we need.
from rob599_hw2.msg import goAction, goGoal, goFeedback, goResult

# This callback will be called when the action is complete.
def done_callback(status, result):
	# The status argument tells you if the action succeeded.  Sometimes actions that did not succeed can
	# return partial results.
	if status == actionlib.GoalStatus.SUCCEEDED:
		rospy.loginfo('Suceeded with result {0}'.format(result.successful))
	else:
		rospy.loginfo('Failed with result {0}. The location you typed in does not exist '.format(result.successful))


# This callback will be called when the action becomes active on the server.  If the server is
# set up to only handle one action at a time, this will let you know when it's actively working
# on your action request.
def active_callback():
	rospy.loginfo('Action is active')


# This callback is called every time the server issues a feedback message.
def feedback_callback(feedback):
	rospy.loginfo('Feedback: {0}'.format(feedback.progress))


if __name__ == '__main__':
	# Get the command line argument, if one exists.  Default to 10
	try:
		arg =str(sys.argv[1])
	except:
		pass
	print(str(arg))
	# Initialize the node
	# rospy.init_node('fib_client', argv=sys.argv[1:])
	rospy.init_node('action_client')
	# Create an action client.  The action name is "fibonacci" and the type is FibonacciAction.  Once we
	# create the client, we're going to wait until it has connected successfully to the server.  This makes
	# sure that we're connected before we start issuing action requests.
	client = actionlib.SimpleActionClient('go_Action', goAction)
	client.wait_for_server()

	# Create a goal message to send to the server.
	goal = goGoal(arg)

	# Send the action request, and register the callbacks.  Note that different requests to the same server
	# can have different callbacks. to make things easier to keep track of.
	client.send_goal(goal, done_cb=done_callback, active_cb=active_callback, feedback_cb=feedback_callback)

	# This will cause the client to wait until there's some result from the server.  If we don't put here,
	# this example will just end before the action has time to execute.  In other applications, you might
	# not wait, but go on do other things, since you'll get notifications of status changes through the
	# callbacks that you registered.
	client.wait_for_result()

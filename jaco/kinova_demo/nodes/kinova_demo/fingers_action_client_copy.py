#! /usr/bin/env python
"""A helper program to test gripper goals for the JACO and MICO arms."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys

import actionlib
import kinova_msgs.msg

import argparse
from pubsub.msg import finger


""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = 'NO_ROBOT_TYPE_DEFINED_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentFingerPosition = [0.0, 0.0, 0.0]

def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/' + prefix + 'driver/fingers_action/finger_positions'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()
    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.WARN('the gripper action timed-out')
        return None


def getCurrentFingerPosition(prefix_):
    # wait to get current position
    topic_address = '/' + prefix_ + 'driver/out/finger_position'
    rospy.Subscriber(topic_address, kinova_msgs.msg.FingerPosition, setCurrentFingerPosition)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.FingerPosition)
    #print 'obtained current finger position '


def setCurrentFingerPosition(feedback):
    global currentFingerPosition
    currentFingerPosition[0] = feedback.finger1
    currentFingerPosition[1] = feedback.finger2
    currentFingerPosition[2] = feedback.finger3

def kinova_robotTypeParser(kinova_robotType_):
    """ Argument kinova_robotType """
    global robot_category, robot_category_version, wrist_type, arm_joint_number, robot_mode, finger_number, prefix, finger_maxDist, finger_maxTurn 
    robot_category = kinova_robotType_[0]
    robot_category_version = int(kinova_robotType_[1])
    wrist_type = kinova_robotType_[2]
    arm_joint_number = int(kinova_robotType_[3])
    robot_mode = kinova_robotType_[4]
    finger_number = int(kinova_robotType_[5])
    prefix = kinova_robotType_ + "_"
    finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
    finger_maxTurn = 6800  # max thread turn for one finger


def unitParser(unit_, finger_value_, relative_):
    """ Argument unit """
    global currentFingerPosition

    if unit_ == 'percent':
        # get absolute value
        finger_turn_command = [x/100.0 * finger_maxTurn for x in finger_value_]
        if relative_:
            finger_turn_absolute_ = [finger_turn_command[i] + currentFingerPosition[i] for i in
                                     range(0, len(finger_value_))]
        else:
            finger_turn_absolute_ = finger_turn_command

        finger_turn_ = finger_turn_absolute_
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]
    else:
        raise Exception("Finger value have to be in turn, mm or percent")

    return finger_turn_, finger_meter_, finger_percent_


def verboseParser(verbose_, finger_turn_):
    """ Argument verbose """
    if verbose_:
        finger_meter_ = [x * finger_maxDist / finger_maxTurn for x in finger_turn_]
        finger_percent_ = [x / finger_maxTurn * 100.0 for x in finger_turn_]
        print('Finger values in turn are: ')
        print(', '.join('finger{:1.0f} {:4.0f}'.format(k[0] + 1, k[1]) for k in enumerate(finger_turn_)))
        print('Finger values in mm are: ')
        print(', '.join('finger{:1.0f} {:2.1f}'.format(k[0]+1, k[1]*1000) for k in enumerate(finger_meter_)))
        print('Finger values in percentage are: ')
        print(', '.join('finger{:1.1f} {:3.1f}%'.format(k[0]+1, k[1]) for k in enumerate(finger_percent_)))


def callback(finger):
    kinova_robotType = 'j2n6s300'
    unit = 'percent'
    finger_value = [finger.x, finger.x, finger.x]
    verbose = False
    relative = False

    kinova_robotTypeParser(kinova_robotType)
    getCurrentFingerPosition(prefix)
    finger_turn, finger_meter, finger_percent = unitParser(unit, finger_value, relative)

    try:
        if finger_number == 0:
            print('Finger number is 0, check with "-h" to see how to use this node.')
            positions = []  # Get rid of static analysis warning that doesn't see the exit()
            exit()
        else:
            positions_temp1 = [max(0.0, n) for n in finger_turn]
            positions_temp2 = [min(n, finger_maxTurn) for n in positions_temp1]
            positions = [float(n) for n in positions_temp2]

        result = gripper_client(positions)
        print('Finger position sent!')
    except rospy.ROSInterruptException:
        print('program interrupted before completion')

    verboseParser(verbose, positions)

if __name__ == '__main__':

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("fingers", finger, callback, queue_size=1)
    rospy.spin()
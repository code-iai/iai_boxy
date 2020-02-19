#!/usr/bin/env python

import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
from sensor_msgs.msg import JointState
import rospy
import numpy
from time import sleep


class TorsoServer(object):

    def __init__(self, name):
        self._action_name = 'whole_body_controller/torso'

        self.torso_joint_name = 'triangle_base_joint'
        self.goal_pos = 0.0
        self.goal_max_vel = 0.01
        self.current_pos = 0.0
        self.got_goal = False

        orig_name = 'omnidrive/giskard_command'
        mapped_name = 'whole_body_controller/velocity_cmd'

        self.omni_client = rospy.Publisher(orig_name, JointState, tcp_nodelay=True,
                                           queue_size=1)
        self.js_sub = rospy.Subscriber('omnidrive/joint_states', JointState, callback=self.js_cb,
                                       queue_size=1, tcp_nodelay=True)

        self.state_pub = rospy.Publisher('{}/state'.format(self._action_name),
                                         JointTrajectoryControllerState, queue_size=10)

        self.trajectory_server = actionlib.SimpleActionServer("{}/follow_joint_trajectory".format(self._action_name),
                                                              FollowJointTrajectoryAction,
                                                              execute_cb=self.execute_traj_cb, auto_start=False)
        self.trajectory_server.start()

    def execute_traj_cb(self, goal):
        """
        :type goal: FollowJointTrajectoryGoal
        """
        # publish info to the console for the user
        rospy.loginfo('%s: Executing a goal:' % self._action_name)
        rospy.loginfo("Goal is:")
        rospy.loginfo(str(goal))

        goal_points = goal.trajectory.points

        if len(goal.trajectory.points) > 0 and len(goal.trajectory.joint_names) and len(goal.trajectory.points):
            try:
                goal.trajectory.joint_names.index(self.torso_joint_name)
                got_goal = True
            except ValueError:
                rospy.logerr("No {} joint in goal message.".format(self.torso_joint_name))
                return

            js_msg = JointState()
            js_msg.name = [self.torso_joint_name]
            timestamp = 0.0

            while got_goal and len(goal_points):
                if self.trajectory_server.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    self.trajectory_server.set_preempted()
                    break
                current_goal = goal_points.pop(0)
                js_msg.position = list(current_goal.positions)
                js_msg.velocity = list(current_goal.velocities)
                js_msg.effort = [0.0]
                self.omni_client.publish(js_msg)
                curr_time = current_goal.time_from_start.secs \
                            + float(current_goal.time_from_start.nsecs) / pow(10, 9)
                sleep(curr_time - timestamp)
                timestamp = curr_time

            # if goal_points list is empty, all goals are met
            if not len(goal_points):
                self.trajectory_server.set_succeeded()
            else:
                rospy.logwarn("%s: client finished before all goals are met." % self._action_name)
        else:
            rospy.logerr("No joints given.")

    def js_cb(self, js_msg):
        """
        :type js_msg: JointState
        """
        try:
            torso_offset = js_msg.name.index(self.torso_joint_name)
        except ValueError:
            rospy.logerr("No {} in omnidrive/joint_states. Can't get position.".format(self.torso_joint_name))
            return
        state_msg = JointTrajectoryControllerState()
        state_msg.joint_names = [self.torso_joint_name]
        self.state_pub.publish(state_msg)
    #
    #     if not self.got_goal:
    #         return
    #
    #     self.current_pos = current_pos
    #     # KISS P controller
    #     gain = 1.0
    #     output = gain * (self.goal_pos - current_pos)
    #
    #     # Limit velocity to the maximum
    #     if abs(output) > self.goal_max_vel:
    #         output = numpy.sign(output) * self.goal_max_vel
    #
    #     # Publish the command for the torso velocity-resolved controller
    #     js_msg = JointState()
    #     js_msg.name = [self.torso_joint_name]
    #     js_msg.position = [0.0]
    #     js_msg.velocity = [output]
    #     js_msg.effort = [0.0]
    #
    #     self.omni_client.publish(js_msg)


if __name__ == '__main__':
    rospy.init_node('iai_giskard_torso_translator')
    server = TorsoServer(rospy.get_name())
    rospy.loginfo('About to spin')
    rospy.spin()


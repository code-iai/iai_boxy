#!/usr/bin/env python

import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
from sensor_msgs.msg import JointState
import rospy
import numpy


class TorsoServer(object):

    def __init__(self, name):
        self._action_name = 'whole_body_controller/torso'

        self.torso_joint_name = 'triangle_base_joint'
        self.goal_pos = 0.0
        self.goal_max_vel = 0.01
        self.current_pos = 0.0
        self.got_goal = False

        self.omni_client = rospy.Publisher('omnidrive/giskard_command', JointState, tcp_nodelay=True,
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

        torso_goal = JointState()

        # Passing the goal info to our class variable
        if len(goal.trajectory.points) > 0 and len(goal.trajectory.points) == len(goal.trajectory.joint_names):
            try:
                j_index = goal.trajectory.joint_names.index(self.torso_joint_name)
            except ValueError:
                rospy.logerr("No {} in goal message.".format(self.torso_joint_name))
                return
            self.goal_pos = goal.trajectory.points[j_index]
            self.got_goal = True
            wait_to_finish = True
            torso_success = False

            while wait_to_finish:
                # Evaluate if we are at the goal
                if abs(self.goal_pos - self.current_pos < 0.0005):
                    # Reached the goal
                    torso_success = True
                    wait_to_finish = False
                    self.got_goal = False

                # check that preempt has not been requested by the client
                if self.trajectory_server.is_preempt_requested():
                    rospy.loginfo('%s: Preempted' % self._action_name)
                    # do something when the goal gets cancelled
                    wait_to_finish = False
                    break

            if torso_success:
                rospy.loginfo('%s: Succeeded' % self._action_name)
                self.trajectory_server.set_succeeded()
        else:
            rospy.logerr("No joints given.")

    def js_cb(self, js_msg):
        """
        :type js_msg: JointState
        """
        if not self.got_goal:
            return
        try:
            torso_offset = js_msg.name.index(self.torso_joint_name)
        except ValueError:
            rospy.logerr("No {} in omnidrive/joint_states. Can't get position.".format(self.torso_joint_name))
            return
        current_pos = js_msg.position[torso_offset]
        self.current_pos = current_pos

        # KISS P controller
        gain = 1.0
        output = gain * (self.goal_pos - current_pos)

        # Limit velocity to the maximum
        if abs(output) > self.goal_max_vel:
            output = numpy.sign(output) * self.goal_max_vel

        # Publish the command for the torso velocity-resolved controller
        js_msg = JointState()
        js_msg.name = [self.torso_joint_name]
        js_msg.position = [0.0]
        js_msg.velocity = [output]
        js_msg.effort = [0.0]

        self.omni_client.publish(js_msg)


if __name__ == '__main__':
    rospy.init_node(rospy.get_name())
    server = TorsoServer(rospy.get_name())
    rospy.loginfo('About to spin')
    rospy.spin()


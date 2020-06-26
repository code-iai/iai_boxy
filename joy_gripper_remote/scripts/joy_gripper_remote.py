#!/usr/bin/env python
# Copyright (c) 2013-2015 Alexis Maldonado Herrera <amaldo@cs.uni-bremen.de>
# Copyright (c) 2015 Georg Bartels <georg.bartels@cs.uni-bremen.de>
# Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import roslib; roslib.load_manifest('joy_gripper_remote')
import rospy

from iai_wsg_50_msgs.msg import PositionCmd
from sensor_msgs.msg import Joy

class JoyGripperRemote( object ):
    '''    '''
    def __init__(self):
        self.left_cmd = rospy.Publisher('/left_arm_gripper/goal_position', PositionCmd, queue_size=5, tcp_nodelay=True)
        self.right_cmd = rospy.Publisher('/right_arm_gripper/goal_position', PositionCmd, queue_size=5, tcp_nodelay=True)
        self.joy_listener = rospy.Subscriber('/joy', Joy, self.joy_callback, tcp_nodelay=True)
        self.open_pos = rospy.get_param('~open_pos', 110.0)
        self.close_pos = rospy.get_param('~close_pos', 6.0)
        self.velocity = rospy.get_param('~velocity', 60.0)
        self.force = rospy.get_param('~force', 30.0)
        
    def joy_callback(self, answer):
        '''
        Callback for controller data from joy, opening/closing the grippers.
        Hold:
            L1: release deadman-switch
            L2: address left gripper
            R2: address right gripper
        Press:
            Triangle: open gripper
            Circle: close gripper
        '''

        #Only do stuff if the dead-man switch (L1) and left-trigger (L2) or right-trigger (R2) is pressed
        if (answer.buttons[4] == 1) :
            # L2
            if (answer.buttons[6] == 1) :
                # Triangle
                if (answer.buttons[2] == 1) :
                    rospy.loginfo("Closing left gripper")
                    self.close(self.left_cmd)
                # Circle
                elif (answer.buttons[1] == 1) :
                    rospy.loginfo("Opening left gripper") 
                    self.open(self.left_cmd)
            # R2
            if (answer.buttons[7] == 1) :
                # Triangle
                if (answer.buttons[2] == 1) :
                    rospy.loginfo("Closing right gripper") 
                    self.close(self.right_cmd)
                # Circle
                elif (answer.buttons[1] == 1) :
                    rospy.loginfo("Opening right gripper") 
                    self.open(self.right_cmd)

    def close(self, pub):
        self.command_pos(self.close_pos, pub)

    def open(self, pub):
        self.command_pos(self.open_pos, pub)

    def command_pos(self, pos, pub):
        pos_cmd = PositionCmd()
        pos_cmd.pos = pos
        pos_cmd.speed = self.velocity
        pos_cmd.force = self.force
        pub.publish(pos_cmd)

def main():
    rospy.init_node('joy_gripper_remote')
    
    my_remote = JoyGripperRemote()

    rospy.spin()

    r = rospy.Rate(5.0)

    while not rospy.is_shutdown():
        r.sleep()
        
        
if __name__ == '__main__':
    main()

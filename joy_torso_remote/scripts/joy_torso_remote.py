#!/usr/bin/env python
# Copyright (c) 2016 Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz)
#
# Author: Alexis Maldonado Herrera <amaldo@cs.uni-bremen.de> 
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

import roslib; roslib.load_manifest('joy_torso_remote')
import rospy

from sensor_msgs.msg import Joy, JointState

vel_scale = 0.08


class TorsoVelInterface(object):
    """
    This class integrates velocities from the joysticks and sends a desired velocity to the torso.
    Release soft_runstop, hold L1, press arrows up/down
    """
    def __init__(self):
        self.joy_torso_vel = 0.0
        self.torso_vel_pub = rospy.Publisher('omnidrive/giskard_command', JointState, tcp_nodelay=True, queue_size=10)
        self.vel_listener = rospy.Subscriber('joy', Joy, self.joy_callback, tcp_nodelay=True)
        
    def joy_callback(self, data):
        """
        Publish the torso's desired movement to the omnidrive.
        :type data: Joy
        """
        # Only do stuff if the dead-man switch L1 is pressed
        # The torso has a watchdog that makes it stop
        if data.buttons[4] == 1:
            self.time_last_vel_cmd = rospy.Time.now().to_sec()            
            self.torso_vel = data.axes[7] * vel_scale

            cmd = JointState()
            cmd.name = ['triangle_base_joint']
            cmd.velocity = [self.torso_vel]
            self.torso_vel_pub.publish(cmd)


def main():
    rospy.init_node('joy_torso_remote')
    
    whr = TorsoVelInterface()
    rospy.spin()

        
if __name__ == '__main__':
    main()

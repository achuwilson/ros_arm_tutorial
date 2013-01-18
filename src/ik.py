#!/usr/bin/env python

"""
    Use inverse kinematics to get joint positions of the left arm necessary to move
    the left finger tip to a given pose. 
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import roslib; roslib.load_manifest('ros_arm_tutorial')
import rospy
from sensor_msgs.msg import JointState
from kinematics_msgs.msg import KinematicSolverInfo, PositionIKRequest
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK, GetPositionIKRequest
import xml.dom.minidom
from math import pi

a=1
import time

class get_fk():
    def __init__(self):
        rospy.init_node("pi_robot_get_ik")

	a=1                
        self.joint_state_pub = rospy.Publisher("joint_states", JointState)
        self.joint_state_update = JointState()
        self.joint_state_update = self.get_joints()
        
        self.rate = 1
        r = rospy.Rate(self.rate)
                        
        rospy.wait_for_service('test_arm_kinematics/get_ik')
        rospy.wait_for_service('test_arm_kinematics/get_ik_solver_info')

        get_ik_proxy = rospy.ServiceProxy('test_arm_kinematics/get_ik', GetPositionIK, persistent=True)
        get_ik_solver_info_proxy = rospy.ServiceProxy('test_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)

        left_arm_solver_info = get_ik_solver_info_proxy()
                    
        self.request = GetPositionIKRequest()
        self.request.timeout = rospy.Duration(5.0)
        self.request.ik_request.pose_stamped.header.frame_id = "base_actuator";
        self.request.ik_request.ik_link_name = "gripper_base";
        self.request.ik_request.pose_stamped.pose.position.x =  0.366796872165
        self.request.ik_request.pose_stamped.pose.position.y = 0.0149175972575
        self.request.ik_request.pose_stamped.pose.position.z = 0.282845877564
        
        self.request.ik_request.pose_stamped.pose.orientation.x = 0.159293225427
        self.request.ik_request.pose_stamped.pose.orientation.y = 0.898180025306
        self.request.ik_request.pose_stamped.pose.orientation.z = 0.0735797897789
        self.request.ik_request.pose_stamped.pose.orientation.w = -0.403093444514

        self.request.ik_request.ik_seed_state.joint_state.name = left_arm_solver_info.kinematic_solver_info.joint_names
        self.request.ik_request.ik_seed_state.joint_state.position = [0]*len(self.request.ik_request.ik_seed_state.joint_state.name )
                
	
        while not rospy.is_shutdown(): 
            try:
                self.response = get_ik_proxy(self.request)
		print self.response.error_code 
#		if( self.response.error_code ==1 ):#== ik_response.error_code.SUCCESS :
#			print "ok" 
                rospy.loginfo(self.response) 
                for joint in self.request.ik_request.ik_seed_state.joint_state.name:
                    self.joint_state_update.position[self.joint_state_update.name.index(joint)] = self.response.solution.joint_state.position[self.response.solution.joint_state.name.index(joint)]
                self.joint_state_update.header.stamp = rospy.Time.now()
                self.joint_state_pub.publish(self.joint_state_update)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            
            r.sleep()
            

    def get_joints(self):
        description = rospy.get_param("robot_description")
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = rospy.get_param("dependent_joints", {})
        
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                    continue
                name = child.getAttribute('name')

                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if name in self.dependent_joints:
                    continue
                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                self.free_joints[name] = joint
                self.joint_list.append(name)

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()

        # Add Free Joints
        for (name, joint) in self.free_joints.items():
            joint_state.name.append(str(name))
            joint_state.position.append(joint['value'])
            
        return joint_state

        
if __name__ == '__main__':
    try:
        get_fk()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down left arm forward kinematics node node...")



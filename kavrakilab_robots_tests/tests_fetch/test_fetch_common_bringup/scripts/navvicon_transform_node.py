#!/usr/bin/env python
"""
Created on April 13, 2018

@author: juandhv (Juan David Hernandez Vega, juandhv@rice.edu)

Purpose: Vicon data transformation to odometry to be used by robot_localization 
"""

# Standard Python imports
from math import *
import numpy as np

# ROS imports
import roslib; roslib.load_manifest('test_fetch_common_bringup')
import rospy
import tf

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class NavigationViconTransform(object):
    """
    Navigation Vicon Transform
    """

    def __init__(self):
        """
        Constructor
        """
        #=======================================================================
        # Initial values
        #=======================================================================
                
        #=======================================================================
        # Services
        #=======================================================================
        
        #=======================================================================
        # Get parameters
        #=======================================================================
        self.vicon_tf_to_vicon_odom_transl_ = rospy.get_param("~vicon_tf_to_vicon_odom_transl", [0.0, 0.0, 0.0])
        self.vicon_tf_to_vicon_odom_rot_ = rospy.get_param("~vicon_tf_to_vicon_odom_rot", [0.0, 0.0, 0.0])
        self.vicon_odom_frame_ = rospy.get_param("~vicon_odom_frame", "base_link")

        #=======================================================================
        # Action lib params
        #=======================================================================
        
        #=======================================================================
        # Subscribers 
        #=======================================================================
        #Navigation data (feedback)
        self.vicon_tf_data_sub_ = rospy.Subscriber("/vicon/tf_data", TransformStamped, self.viconTfDataSubCallback, queue_size = 1)
        
        #=======================================================================
        # Publishers
        #=======================================================================
        self.vicon_odom_pub_ = rospy.Publisher("/odometry/vicon", Odometry, queue_size = 1)

        #=======================================================================
        # Register handler to be called when rospy process begins shutdown.
        #=======================================================================
        #rospy.on_shutdown(self.shutdownHook)
        
        #=======================================================================
        # Spin
        #=======================================================================
        rospy.spin()
        return

    
    def __del__(self):
        """
        Destructor
        """
        return
    
    
    def viconTfDataSubCallback(self, transform_stamped_msg):
        """
        Callback to receive Vicon TF data
        """
        vicon_odom_msg = Odometry()
        vicon_odom_msg.header.stamp = transform_stamped_msg.header.stamp
        vicon_odom_msg.header.frame_id = transform_stamped_msg.header.frame_id
        vicon_odom_msg.child_frame_id = self.vicon_odom_frame_
        
        self.transform_stamped_msg = transform_stamped_msg
        
        r, p, y = tf.transformations.euler_from_quaternion([transform_stamped_msg.transform.rotation.x,
                                                            transform_stamped_msg.transform.rotation.y,
                                                            transform_stamped_msg.transform.rotation.z, 
                                                            transform_stamped_msg.transform.rotation.w])
        r += self.vicon_tf_to_vicon_odom_rot_[0]
        p += self.vicon_tf_to_vicon_odom_rot_[1]
        y += self.vicon_tf_to_vicon_odom_rot_[2]
        qx, qy, qz, qw = tf.transformations.quaternion_from_euler(r, p, y)
        
        vicon_odom_msg.pose.pose.position.x = transform_stamped_msg.transform.translation.x
        vicon_odom_msg.pose.pose.position.y = transform_stamped_msg.transform.translation.y
        vicon_odom_msg.pose.pose.position.z = transform_stamped_msg.transform.translation.z
        vicon_odom_msg.pose.pose.orientation.x = qx
        vicon_odom_msg.pose.pose.orientation.y = qy
        vicon_odom_msg.pose.pose.orientation.z = qz
        vicon_odom_msg.pose.pose.orientation.w = qw
        
        qmatrix = tf.transformations.quaternion_matrix([transform_stamped_msg.transform.rotation.x,
                                                        transform_stamped_msg.transform.rotation.y,
                                                        transform_stamped_msg.transform.rotation.z,
                                                        transform_stamped_msg.transform.rotation.w])
        qmatrix[0][3] = transform_stamped_msg.transform.translation.x
        qmatrix[1][3] = transform_stamped_msg.transform.translation.y
        qmatrix[2][3] = transform_stamped_msg.transform.translation.z
        
        vicon_pos = np.array(self.vicon_tf_to_vicon_odom_transl_ + [1.0])
        vicon_pos.shape = (4,1)
        vicon_transformed_pos = np.dot(qmatrix, vicon_pos)
        
        vicon_odom_msg.pose.pose.position.x = vicon_transformed_pos[0]
        vicon_odom_msg.pose.pose.position.y = vicon_transformed_pos[1]
        vicon_odom_msg.pose.pose.position.z = vicon_transformed_pos[2]
        
        self.vicon_odom_pub_.publish(vicon_odom_msg)
        
        return
        
    def shutdownHook(self):
        """   
        shutdownHook: gets called on shutdown and cancels all ongoing pilot goals     
        """
        rospy.loginfo("%s: shutting down node", rospy.get_name())
        return
    
if __name__ == '__main__':
    try:
        rospy.init_node('navvicon_transform', log_level=rospy.INFO) #log_level=rospy.DEBUG
        rospy.loginfo("%s: Navigation Vicon Transform", rospy.get_name())
        
        #=======================================================================
        # Init: create navvicon_transform
        #=======================================================================
        navvicon_transform = NavigationViconTransform()
            
    except rospy.ROSInterruptException:
        pass
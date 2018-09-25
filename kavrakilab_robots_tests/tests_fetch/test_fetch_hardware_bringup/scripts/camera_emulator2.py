#!/usr/bin/env python
"""
Created on March 25, 2016

@author: juandhv (Juan David Hernandez Vega, juandhv@rice.edu)

Purpose: Camera emulator
"""

# Standard Python imports
from math import *
import numpy as np
import actionlib
from collections import deque

# Debug control variables
DEBUG_OCTOMAP_VIS = False

# ROS imports
import roslib; roslib.load_manifest('test_fetch_hardware_bringup')
import rospy
from geometry_msgs.msg import Point, TransformStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty, EmptyRequest
from visualization_msgs.msg import Marker
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import tf

from nav_msgs.msg import Odometry

class CameraEmulator(object):
    """
    Camera Emulator
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

        #=======================================================================
        # Action lib params
        #=======================================================================
        
        #=======================================================================
        # Subscribers 
        #=======================================================================
        #Navigation data (feedback)
        self.link_states_sub_ = rospy.Subscriber("/vicon/Fetch/Fetch", TransformStamped, self.linkStatesSubCallback, queue_size = 1)
        self.link_states_available_ = False
        
        #=======================================================================
        # Publishers
        #=======================================================================
        self.base_link_odom_ = rospy.Publisher("/vicon/base_link_odom", Odometry, queue_size = 1)

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
    
    
    def linkStatesSubCallback(self, transform_stamped_msg):
        """
        Callback to receive Odometry message
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = transform_stamped_msg.header.stamp
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = "base_link"
        
        self.transform_stamped_msg = transform_stamped_msg
        if self.link_states_available_ == False:
            self.link_states_available_ = True
        
        #quaternion = geometry_msgs.msg.Quaternion(transform_stamped_msg.transform.rotation.x, transform_stamped_msg.transform.rotation.y, transform_stamped_msg.transform.rotation.z, transform_stamped_msg.transform.rotation.w)
        r, p, y = tf.transformations.euler_from_quaternion([transform_stamped_msg.transform.rotation.x, transform_stamped_msg.transform.rotation.y, transform_stamped_msg.transform.rotation.z, transform_stamped_msg.transform.rotation.w])
        y -= 1.5707963
        qx, qy, qz, qw = tf.transformations.quaternion_from_euler(r, p, y)
        
        odom_msg.pose.pose.position.x = transform_stamped_msg.transform.translation.x
        odom_msg.pose.pose.position.y = transform_stamped_msg.transform.translation.y# + 0.23
        odom_msg.pose.pose.position.z = transform_stamped_msg.transform.translation.z# - 0.27
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        #print transform_stamped_msg.transform
        qmatrix = tf.transformations.quaternion_matrix([transform_stamped_msg.transform.rotation.x, transform_stamped_msg.transform.rotation.y, transform_stamped_msg.transform.rotation.z, transform_stamped_msg.transform.rotation.w])
        qmatrix[0][3] = transform_stamped_msg.transform.translation.x
        qmatrix[1][3] = transform_stamped_msg.transform.translation.y
        qmatrix[2][3] = transform_stamped_msg.transform.translation.z
        
        v = np.array([-0.02, -0.23, -0.26, 1])
        v.shape = (4,1)
        new_v = np.dot(qmatrix, v)
        
        odom_msg.pose.pose.position.x = new_v[0]
        odom_msg.pose.pose.position.y = new_v[1]
        odom_msg.pose.pose.position.z = new_v[2]
        
        self.base_link_odom_.publish(odom_msg)
        
        
        return
        
    def shutdownHook(self):
        """
        wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww    
        shutdownHook: gets called on shutdown and cancels all ongoing pilot goals (Enric)
        wwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww    
        """
        rospy.loginfo("%s: shutting down node", rospy.get_name())
        self.wp_act_lib_action_client_.cancel_all_goals()
        self.sect_act_lib_action_client_.cancel_all_goals()
        self.path_act_lib_action_client_.cancel_all_goals()
#         rospy.signal_shutdown("manual shutdown")
    
if __name__ == '__main__':
    try:
        rospy.init_node('camera_emulator', log_level=rospy.INFO) #log_level=rospy.DEBUG
        rospy.loginfo("%s: Camera Emulator", rospy.get_name())
        
        #=======================================================================
        # Init: create camera_emulator
        #=======================================================================
        camera_emulator = CameraEmulator()
#         camera_emulator.submerge()
#         rospy.sleep(10.0)
        camera_emulator.run()
        
        #rospy.spin()
            
    except rospy.ROSInterruptException:
        pass
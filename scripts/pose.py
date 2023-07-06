#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
import tf.transformations as tft
from jetbot_ros.msg import Pose

def tf_to_pose():
    
    # Pose message publisher
    pose_publisher = rospy.Publisher('pose_topic', Pose, queue_size=2)
    
    # Initialize the ROS node
    rospy.init_node('pose_publisher', anonymous=True)

<<<<<<< HEAD
class Object:

    def __init__(self, form, x, y, color) -> None:
        self.form: str = form
        self.x: float = x
        self.y: float = y
        self.color: str = color


class Pose:

    def __init__(self, x, y, angle) -> None:
        self.x: float = x
        self.y: float = y
        self.angle: float = angle


def get_pose():
    # Initialize node
    rospy.init_node('tf_listener_node', anonymous=True)
=======
>>>>>>> df2f7ef2aa618a55a8fe17d20afa8922977c0c7a
    # TF message listener
    tf_listener = tf.TransformListener()
    
    # Publishing rate
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        try:
            # Lookup the transformation from the CSI camera frame to the fixed map 
            (trans, q) = tf_listener.lookupTransform('map', 'csi://0', rospy.Time(0))
            
            # Convert the quaternion to euler angles, rotation matrix
            euler = tft.euler_from_quaternion(q)
            #rot = tft.quaternion_matrix(q)
             
            # (x,y) robot pose wrt arena
            # camera orientation angle theta wrt the arena's y-axis
            x = trans[0]
            y = trans[1]
            theta_radians = euler[2]
            # Convert the angle to degrees
            theta = math.degrees(theta_radians)
            
            # Create Pose message
            pose_msg = Pose()
            pose_msg.x = x
            pose_msg.y = y
            pose_msg.theta = theta
            pose_msg.translation = trans
            pose_msg.rotation = q
          
            # Publish the Pose message
            pose_publisher.publish(pose_msg)
            print(pose_msg)
            
            rate.sleep()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('TF lookup exception occurred.')
                   
if __name__ == '__main__':
    try:
        tf_to_pose()
    except rospy.ROSInterruptException:
        pass
    
    # Spin to keep the node alive
    rospy.spin()
    
    
    

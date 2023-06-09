import rospy
import tf
import geometry_msgs.msg
import math
import numpy as np

def pose():
    # Initialize node
    rospy.init_node('tf_listener_node', anonymous=True)
    # TF message listener
    tf_listener = tf.TransformListener()
    
    rate = rospy.Rate(10)  

    while not rospy.is_shutdown():
        try:
            # Lookup the transformation from the CSI camera frame to the fixed map frame
            (trans, rot) = tf_listener.lookupTransform('map', 'csi://0', rospy.Time(0))
            # Print the translation and rotation information
            print('Translation:', trans)
            print('Rotation:', rot)
            
            ## GET POSE, ANGLES##
            x = trans[0]
            y = trans[1]
            theta = math.acos(((rot[0][0] + rot[1][1] + rot[2][2]) - 1) / 2)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('TF lookup exception occurred.')
    return x,y,theta
    
if __name__ == '__main__':
    try:
        tf_listener()
        rate.sleep()
    except rospy.ROSInterruptException:
        pass

import rospy
import tf
import geometry_msgs.msg

def tf_listener():
    rospy.init_node('tf_listener_node', anonymous=True)

    tf_listener = tf.TransformListener()

    rate = rospy.Rate(10)  # Adjust the rate as per your requirement

    while not rospy.is_shutdown():
        try:
            # Lookup the transformation from the CSI camera frame to the fixed map frame
            (trans, rot) = tf_listener.lookupTransform('map', 'csi://0', rospy.Time(0))

            # Print the translation and rotation information
            print('Translation:', trans)
            print('Rotation:', rot)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('TF lookup exception occurred.')

        rate.sleep()

if __name__ == '__main__':
    try:
        tf_listener()
    except rospy.ROSInterruptException:
        pass

# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Initialize the ROS Node named 'cv_tools'
rospy.init_node('cv_tools', anonymous=True)
 
# Initialize CvBridge class
bridge = CvBridge()

# Define a callback for the Image message
def image_callback(img_msg):
    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    # Show the converted image
    cv2.imshow("Image", cv_image)
    return cv_image
    
# Read object's color
def get_color(box):
    # Convert to grayscale
    gray = cv2.cvtColor(box, cv2.COLOR_BGR2GRAY)   
    # Apply binary thresholding
    ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY) 
    # Find contours
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Calculate the mean color inside the contour
    mask = np.zeros(img.shape[:2], dtype=np.uint8)
    mean_color = cv2.mean(box, mask=mask)[:3]
   
    colors = {
        'green': ([30, 70, 30], [70, 255, 70]),
        'blue': ([30, 30, 70], [70, 70, 255]),
        'red': ([70, 30, 30], [255, 70, 70]),
        'yellow': ([70, 70, 30], [255, 255, 70]),
        'purple': ([70, 30, 70], [255, 70, 255]),
        'orange': ([70, 50, 30], [255, 150, 70])
    }
    
    # Determine the color 
    colors = None
    for color, (lower, upper) in colors.items():
        if np.all(mean_color >= lower) and np.all(mean_color <= upper):
            colors = color
            break
    return color
    
    

# Initalize a subscriber 
sub_image = rospy.Subscriber("/camera/image_raw", Image, image_callback)

# Initialize an OpenCV Window
cv2.namedWindow("Image", 1)

while not rospy.is_shutdown():
    rospy.spin()

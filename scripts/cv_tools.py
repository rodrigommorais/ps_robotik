import numpy as np
import cv2
import rospy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from jetbot_ros.msg import Object
bridge = CvBridge()

# Determine object's color
def color(cv_image, x_center, y_center, w, h):
    # Define color ranges 
    colors = {
        'Red1': [1,10],
        'Red2': [150,179],
        'Orange': [10,30],
        'Yellow': [30,60],
        'Green': [60,81],
        'Blue': [81,120],
        'Purple': [120,150]
        
    }
    # Compute bounding box corners' coordinate (in pix)
    x_tl = np.int0(x_center - w*0.4)
    y_tl = np.int0(y_center - h*0.4)
    x_br = np.int0(x_tl + w*0.75)
    y_br = np.int0(y_tl + h*0.75)
    
    # Compute mean HSV value of the region of interest
    roi = cv_image[y_tl:y_br, x_tl:x_br]
    roi = np.rot90(roi,-1)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mean_color = np.mean(hsv, axis=(0, 1)).astype(int)
    print(mean_color)
    
    
    # Define a threshold range around the mean HSV 
    for color_name, target_hsv in colors.items():
        #h_diff = min(np.abs(mean_color[0] - target_hsv[0]), 179 - np.abs(mean_color[0] - target_hsv[0]))
        #if h_diff <= h_threshold:
        if mean_color[0] in range (target_hsv[0],target_hsv[1]):
            return color_name 
            break
            

def img_callback(msg):
    global image
    try:
        # Convert ROS image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")     
        image = cv_image
    except CvBridgeError as e:
        rospy.logerr(e)
        
        
def detect(detect_msg):
    area = []
    detections = detect_msg.detections
    for detection in detections:
        w = detection.bbox.size_x
        h = detection.bbox.size_y
        area.append(w*h)
    i = np.argmin(area)
    
    shape = detections[i].results[0].id
    if shape==1:
        shape = 'sphere'
    elif shape==2:
        shape = 'cube'
        
    score = detections[i].results[0].score
    x_center = detections[i].bbox.center.x
    y_center = detections[i].bbox.center.y
    w = detections[i].bbox.size_x
    h = detections[i].bbox.size_y
    find_object_position(x_center, y_center,w,h)
    
    rospy.Subscriber("/camera/image_raw", Image, img_callback)
    print(color(image,x_center, y_center, w, h))
    return shape, score
        

def find_object_position(x_center, y_center,w,h):
    # Cube length in pixel
    d_pix = np.minimum(w,h)
    # Cube length in meters
    d_m = 0.035
    
    # Cube centeroid
    centeroid = (x_center, y_center)
   
    # Extract the camera intrinsic matrix 
    with open("/home/jetbot11/workspace/catkin_ws/src/jetbot_ros/ost.yaml", "r") as file:
        data = yaml.load(file, Loader=yaml.Loader)
    matrix = data['camera_matrix']
    intrinsic_matrix = np.array(matrix['data']).reshape(matrix['rows'],matrix['cols'])
    fx = intrinsic_matrix[0,0]
    fy = intrinsic_matrix[1,1]
    cx = intrinsic_matrix[2,0]
    cy = intrinsic_matrix[2,1]
    # Extract distortion coefficients
    coeff = data['distortion_coefficients']
    distortion_coefficients = np.array(coeff['data']).reshape(coeff['rows'],coeff['cols'])
  
    # Undistort the image coordinates
    centeroid = cv2.undistortPoints(centeroid, intrinsic_matrix, distortion_coefficients)
    
    # Map image coordinates to camera coordinates
    z_cam = (fx*d_m)/d_pix
    x_cam = ((x_center-cx)*z_cam)/fx
    y_cam = ((y_center-cy)*z_cam)/fy
    
    # Map camera coordinates to world coordinates
    print(z_cam)
   
      
if __name__ == "__main__":
    rospy.init_node("detection")
    rospy.Subscriber("/detectnet/detections", Detection2DArray, detect)
  
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    rospy.spin()

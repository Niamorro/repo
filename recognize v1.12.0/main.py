import math
from threading import Lock
import cv2
import numpy as np
import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover.srv import SetLEDEffect

# Service proxies
get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
navigate = rospy.ServiceProxy("navigate", srv.Navigate)
navigate_global = rospy.ServiceProxy("navigate_global", srv.NavigateGlobal)
set_position = rospy.ServiceProxy("set_position", srv.SetPosition)
set_velocity = rospy.ServiceProxy("set_velocity", srv.SetVelocity)
set_attitude = rospy.ServiceProxy("set_attitude", srv.SetAttitude)
set_rates = rospy.ServiceProxy("set_rates", srv.SetRates)
land = rospy.ServiceProxy("land", Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

# Global variables
detected_objects = []
DUPLICATE_THRESHOLD = 0.7

telem_lock = Lock()
bridge = CvBridge()

# Debug publisher
debug_pub = None
Start_position = None

# Color ranges in HSV
color_ranges = {
    'red': ([0, 100, 100], [10, 255, 255]),
    'blue': ([95, 60, 60], [110, 255, 255]),
    'green': ([60, 50, 50], [80, 255, 255]),
    'yellow': ([20, 100, 100], [40, 255, 255]),
    'pink': ([145, 120, 100], [165, 255, 255])
}

# Color RGB values
color_rgb = {
    'red': (255, 0, 0),
    'blue': (0, 0, 255),
    'green': (0, 255, 0),
    'yellow': (255, 255, 0),
    'pink': (255, 20, 147)
}

def led_controller():
    """Проверяет расстояние до обнаруженных объектов и управляет светодиодами"""
    current_telem = get_telemetry_locked(frame_id='aruco_map')
    current_pos = (current_telem.x, current_telem.y)
    MAX_LED_DISTANCE = 1.0
    
    for obj in detected_objects:
        distance = math.sqrt((obj['x'] - current_pos[0])**2 + (obj['y'] - current_pos[1])**2)
        if distance < MAX_LED_DISTANCE:
            if obj['color'] in color_rgb:
                r, g, b = color_rgb[obj['color']]
                set_effect(effect='fill', r=r, g=g, b=b)
                return
    
    set_effect(effect='fill', r=0, g=0, b=0)

def duplicate_detection(new_object, detected_objects):
    """Check if the object is a duplicate based on distance threshold"""
    for obj in detected_objects:
        if obj['color'] == new_object['color']:
            distance = math.sqrt((obj['x'] - new_object['x'])**2 + (obj['y'] - new_object['y'])**2)
            if distance < DUPLICATE_THRESHOLD:
                return True
    return False

def image_callback(data):
    global debug_pub
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    
    height, width = cv_image.shape[:2]
    crop_width = int(width * 0.1)
    crop_height = int(height * 0.1)
    x1 = width//2 - crop_width//2
    x2 = width//2 + crop_width//2
    y1 = height//2 - crop_height//2
    y2 = height//2 + crop_height//2
    
    cropped = cv_image[y1:y2, x1:x2]
    debug_image = cv_image.copy()
    cv2.rectangle(debug_image, (x1, y1), (x2, y2), (255, 255, 255), 2)
    
    hsv = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)
    telem = get_telemetry_locked(frame_id='aruco_map')
    current_position = (telem.x, telem.y)

    cv2.putText(debug_image, f"Pos: ({current_position[0]:.2f}, {current_position[1]:.2f})", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    for color_name, (lower, upper) in color_ranges.items():
        lower = np.array(lower)
        upper = np.array(upper)
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 50:
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    cx_orig = cx + x1
                    cy_orig = cy + y1
                    
                    object_info = {
                        'color': color_name,
                        'x': current_position[0],
                        'y': current_position[1]
                    }
                    
                    if not duplicate_detection(object_info, detected_objects):
                        detected_objects.append(object_info)
                        print(f"Detected new {color_name} object at position {current_position}")
                        led_controller()
                        
                        cv2.drawContours(debug_image[y1:y2, x1:x2], [contour], -1, (0, 255, 0), 2)
                        cv2.circle(debug_image, (cx_orig, cy_orig), 5, (0, 0, 255), -1)
                        cv2.putText(debug_image, f"{color_name} ({current_position[0]:.2f}, {current_position[1]:.2f})",
                                  (cx_orig - 20, cy_orig - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    else:
                        cv2.drawContours(debug_image[y1:y2, x1:x2], [contour], -1, (128, 128, 128), 1)
                        cv2.putText(debug_image, "Duplicate", (cx_orig - 20, cy_orig - 20),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 1)

    if debug_pub is not None:
        debug_msg = bridge.cv2_to_imgmsg(debug_image, 'bgr8')
        debug_pub.publish(debug_msg)

def generate_report():
    with open('report_fly.txt', 'w') as f:
        for i, obj in enumerate(detected_objects, 1):
            f.write(f"object {i}: {obj['color']} {obj['x']:.2f} {obj['y']:.2f}\n")
    print("Report generated successfully")

def get_telemetry_locked(frame_id=""):
    with telem_lock:
        telem = get_telemetry(frame_id=frame_id)
    return telem

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    print(f"Going to x: {x} y: {y}")
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry_locked(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
    print("Target reached")

def clean_up_detections():
    """Clean up the detected objects list by removing duplicates"""
    global detected_objects
    cleaned_objects = []
    for obj in detected_objects:
        if not duplicate_detection(obj, cleaned_objects):
            cleaned_objects.append(obj)
    detected_objects = cleaned_objects

def main_flight(z=1.5):
    global debug_pub, start_position
    
    rospy.init_node('flight_node')
    debug_pub = rospy.Publisher('/F_I_debug', Image, queue_size=1)
    image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
    
    rospy.sleep(1)
    
    print("Taking off...")
    navigate(x=0, y=0, z=z, speed=0.5, frame_id='body', auto_arm=True, yaw=float('nan'))
    rospy.sleep(2)
    print("Took off")
    
    navigate_wait(x=0, y=0, z=z, speed=0.5, frame_id='body')
    rospy.sleep(2)
    
    start_position = get_telemetry_locked(frame_id='aruco_map')
    print(f"Saved takeoff position: x={start_position.x}, y={start_position.y}")
    
    navigate_wait(x=0, y=0, z=z, speed=0.5)
    print("Start position")

    rospy.Timer(rospy.Duration(0.1), lambda _: led_controller())

    direction_forward = True
    for x in range(0, 4, 1):
        if direction_forward:
            navigate_wait(x=x, y=0, z=z)
            navigate_wait(x=x, y=5, z=z)
        else:
            navigate_wait(x=x, y=5, z=z)
            navigate_wait(x=x, y=0, z=z)
        direction_forward = not direction_forward

    print(f"Returning to takeoff position: x={start_position.x}, y={start_position.y}")
    navigate_wait(x=start_position.x, y=start_position.y, z=z, frame_id="aruco_map")
    
    clean_up_detections()
    generate_report()
    
    print("Landing at takeoff position")
    land()
if __name__ == "__main__":
    main_flight()
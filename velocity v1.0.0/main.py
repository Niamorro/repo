#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import srv
from std_srvs.srv import Trigger
import math
import time

# Initialize ROS
rospy.init_node('car_velocity_detector')
bridge = CvBridge()

# Create service proxies for drone control
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

# Create debug image publisher
debug_pub = rospy.Publisher('debug', Image, queue_size=10)

# Dictionary to store detected cars and their velocities
detected_cars = {}

# Dictionary to store previous car positions and timestamps
prev_positions = {}
prev_times = {}

# Flight height
z = 1.5

# HSV color ranges for detection
color_ranges = {
    'red': ([0, 100, 100], [10, 255, 255]),
    'blue': ([110, 100, 100], [130, 255, 255]),
    'green': ([50, 100, 100], [70, 255, 255])
}

# Function to wait until reaching a point
def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    rospy.loginfo(f"Moving to point: x={x}, y={y}, z={z}")
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)
    rospy.loginfo("Point reached")

# Function to process image frames
def image_callback(data):
    try:
        # Get the image
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        debug_image = cv_image.copy()
        
        # Get current telemetry
        telem = get_telemetry(frame_id='aruco_map')
        current_time = time.time()
        
        # Add position info to debug image
        cv2.putText(debug_image, f"Pos: ({telem.x:.2f}, {telem.y:.2f})", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Convert image to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Process each color
        for color, (lower, upper) in color_ranges.items():
            lower = np.array(lower)
            upper = np.array(upper)
            
            # Create mask for current color
            mask = cv2.inRange(hsv, lower, upper)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter small contours
                if area > 100:
                    # Find contour center
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Draw contour and center on debug image
                        cv2.drawContours(debug_image, [contour], -1, (0, 255, 0), 2)
                        cv2.circle(debug_image, (cx, cy), 5, (0, 0, 255), -1)
                        
                        # Calculate approximate car position in world coordinates
                        # This is a simple approximation - in reality, you'd need proper camera calibration
                        car_x = telem.x + (cx - cv_image.shape[1]/2) / 100
                        car_y = telem.y + (cy - cv_image.shape[0]/2) / 100
                        
                        # Calculate distance to all previously detected cars of the same color
                        min_distance = float('inf')
                        for car_id, (detected_color, _) in detected_cars.items():
                            if detected_color == color and car_id.startswith(f"{color}_"):
                                # Extract coordinates from car_id
                                parts = car_id.split('_')
                                if len(parts) >= 3:
                                    try:
                                        detected_x = float(parts[1]) / 10
                                        detected_y = float(parts[2]) / 10
                                        distance = math.sqrt((car_x - detected_x)**2 + (car_y - detected_y)**2)
                                        min_distance = min(min_distance, distance)
                                    except ValueError:
                                        pass
                        
                        # If car is far enough from all previously detected cars (> 1.0m), consider it new
                        if min_distance > 1.0:
                            # Create unique ID for the car based on color and position
                            car_id = f"{color}_{int(car_x*10)}_{int(car_y*10)}"
                            
                            # Calculate velocity if we have previous position for this color
                            velocity = 0
                            if color in prev_positions and color in prev_times:
                                prev_x, prev_y = prev_positions[color]
                                prev_time = prev_times[color]
                                
                                # Calculate velocity
                                dx = car_x - prev_x
                                dy = car_y - prev_y
                                dt = current_time - prev_time
                                
                                if dt > 0:
                                    distance = math.sqrt(dx**2 + dy**2)
                                    velocity = distance / dt  # m/s
                            
                            # Save current position and time
                            prev_positions[color] = (car_x, car_y)
                            prev_times[color] = current_time
                            
                            # If velocity is greater than 0, save car info
                            if velocity > 0:
                                detected_cars[car_id] = (color, velocity)
                                
                                # Display info on debug image and log
                                info_text = f"{color}: {velocity:.2f} m/s"
                                cv2.putText(debug_image, info_text, (cx, cy - 10), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                                
                                rospy.loginfo(f"Detected car: color={color}, velocity={velocity:.2f} m/s")
                        else:
                            # If car is already detected, show saved info
                            cv2.putText(debug_image, "Already detected", (cx, cy - 10), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (128, 128, 128), 2)
        
        # Publish debug image
        debug_pub.publish(bridge.cv2_to_imgmsg(debug_image, 'bgr8'))
        
    except Exception as e:
        rospy.logerr(f"Error processing image: {e}")

# Function to create report
def create_report():
    # Filter out duplicate detections
    unique_cars = {}
    
    # Group by color and find average velocity for each color
    for car_id, (color, velocity) in detected_cars.items():
        if color not in unique_cars:
            unique_cars[color] = []
        unique_cars[color].append(velocity)
    
    # Write to report file
    with open('report.txt', 'w') as file:
        for color, velocities in unique_cars.items():
            avg_velocity = sum(velocities) / len(velocities)
            file.write(f"{color};{avg_velocity:.2f}\n")
    
    total_unique_colors = len(unique_cars)
    rospy.loginfo(f"Report created. Detected {total_unique_colors} unique cars.")

# Main flight function
def flight_mission():
    rospy.loginfo("Starting flight mission")
    
    # Subscribe to camera feed
    image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)
    
    # Take off
    rospy.loginfo("Taking off...")
    navigate(x=0, y=0, z=1.0, frame_id='body', auto_arm=True)
    rospy.sleep(3)
    
    # Move to aruco map coordinates
    rospy.loginfo("Transitioning to aruco map...")
    navigate_wait(x=0, y=0, z=z, frame_id='aruco_map')
    rospy.sleep(2)
    
    # Zigzag flight pattern
    rospy.loginfo("Starting zigzag pattern...")
    direction_forward = True
    for x in range(0, 4, 1):
        if direction_forward:
            navigate_wait(x=x, y=0, z=z)
            navigate_wait(x=x, y=5, z=z)
        else:
            navigate_wait(x=x, y=5, z=z)
            navigate_wait(x=x, y=0, z=z)
        direction_forward = not direction_forward
    
    # Return to start and land
    rospy.loginfo("Returning to start...")
    navigate_wait(x=0, y=0, z=z)
    rospy.loginfo("Landing...")
    land()
    
    # Create report
    create_report()

if __name__ == '__main__':
    try:
        flight_mission()
    except rospy.ROSInterruptException:
        pass
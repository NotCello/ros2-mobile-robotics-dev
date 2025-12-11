import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from aruco_opencv_msgs.msg import ArucoDetection
import math
import time
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np

DEBUG = False

def quaternion_to_yaw(x, y, z, w):
    """Convert quaternion to yaw (rotation around Z axis)"""
    return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

def normalize_angle(angle):
    """Normalize the angle between [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))

def is_centered_pixel(center, img_shape, pixel_thresh=9):
    """Check if the marker's center is centered in the camera view"""
    _, w = img_shape[:2]
    cx_img = w/2.0
    dx = center[0] - cx_img
    return abs(dx) <= pixel_thresh
            

class ScanMarkers(Node):
    def __init__(self):
        super().__init__('scan_markers')
        self.markers = {}                                               # [id marker] = Robot's angle after detecting the marker
        self.centered_counts = {}                                       # [id_marker] = number of consecutive frames centered
        self.yaw = None                                                 # Current robot yaw orientation
        self.EXCPECTED_MARKERS = 5                                      # Number of markes expected
        self.REQUIRED_CONSECUTIVE = 3                                   # Number of consecutive frames to ensure the marker is centered
        self.bridge = CvBridge()
        self.cv_image = None                                            # Used to save the image already converted into a cv image
        self.header = None
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.image_pub = self.create_publisher(CompressedImage, 'camera/image_with_circle', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom,
            100
        )
        self.markers_sub = self.create_subscription(
            ArucoDetection,
            '/aruco_detections',
            self.marker_detection,
            100
        )
        self.image_sub = self.create_subscription(                      # Subscriber to the original camera topic
            CompressedImage, 
            'camera/rgb/image_raw/compressed', 
            self.image_callback, 
            10
        )

    def odom(self, msg):
        """Perform the robot yaw orientation"""
        q = msg.pose.pose.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        
    def marker_detection(self, msg):
        """
        Perform the marker detection by retrieving the ID and position. Calculate the angle of the camera between the x-axis and z-axis, 
        then determine the normalized global yaw. Compute the score as the absolute value of the camera angle. 
        Update the dictionary continuously, keeping the entry with the lowest score, which indicates that the marker is centered in the camera view
        """
        if not msg.markers or self.yaw is None:
            return
    
        for marker in msg.markers:
            marker_id = int(marker.marker_id)   
            if marker_id==10 or marker_id==12 or marker_id==14 or marker_id==16 or marker_id==18:
                    # Ignore these markers
                px = marker.pose.position.x
                py = marker.pose.position.y
                pz = marker.pose.position.z
                if px == 0 and py == 0 and pz == 0:
                    continue
                
                try:
                    angle_cam = math.atan2(px, pz)
                    camera_mounting_offset = 0.0
                    global_yaw = normalize_angle(self.yaw + angle_cam + camera_mounting_offset)
                    score = abs(angle_cam)      # smaller = more centered

                    prev = self.markers.get(marker_id)
                    if prev is None:
                        self.get_logger().info(f'Marker {marker_id} detected: {len(self.markers) + 1}/{self.EXCPECTED_MARKERS}')
                    # Keep the best estimation
                    if prev is None or score < prev[1]:                    
                        self.markers[marker_id] = (global_yaw, score)
                        if DEBUG:
                            self.get_logger().info(f'Best pose for marker {marker_id} updated: robot={self.yaw} position={global_yaw} angle_cam={math.degrees(angle_cam):.1f}° score={score:.3f}')
                except Exception as e:
                    if DEBUG:
                        self.get_logger().info(f'Exception raised by the marker detection: {e}')
                    continue
                
    def image_callback(self, msg):
        """Convert and save the picture into an opencv image"""
        try:
            # Convert ROS Image to OpenCV image
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.header = msg.header
        except Exception as e:
            self.get_logger().error(f'Exception raised by the image conversion: {e}')
                
    def safe_publish(self, publisher, msg):
        """Publish twist only if context still valid"""
        try:
            if rclpy.ok():
                publisher.publish(msg)
        except Exception as e:
            self.get_logger().warning(f'Exception raised while publishing: {e}')
            
    def draw_circle(self, id):
        """Draw a circle around the marker so that it passes through the corners"""
        if self.cv_image is None:
            self.get_logger().warning(f'No image has been saved')
            return
        
        try:
            cv2.namedWindow(f'marker-{id}', cv2.WINDOW_AUTOSIZE)
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
            parameters = cv2.aruco.DetectorParameters_create()
            # detectMarkers -> corners, ids, rejectedCandidates
            corners, ids, rejected = cv2.aruco.detectMarkers(self.cv_image, aruco_dict, parameters=parameters)
            if ids is None or len(ids) == 0:
                self.get_logger().warning(f'No Aruco markers detected')
            else:
                for i, c in enumerate(corners):
                    marker_id = int(ids[i][0])
                    pts = c.reshape((4, 2))
                    center = pts.mean(axis=0)
                    dists = np.linalg.norm(pts - center, axis=1)
                    radius = int(np.ceil(dists.max()))
                    center_int = (int(round(center[0])), int(round(center[1])))
                    # Draw the red circle
                    cv2.circle(self.cv_image, center_int, radius, (0, 0, 255), 2)
                    # Draw a little dot in the middle of the marker
                    cv2.circle(self.cv_image, center_int, 3, (0, 0, 255), -1)
                    # Set the id above the marker
                    cv2.putText(self.cv_image, f'ID={marker_id}',(center_int[0], center_int[1] + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            # Show the image
            cv2.imshow(f'marker-{id}', self.cv_image)
            cv2.waitKey(20)
        except Exception as e:
            self.get_logger().error(f'Exception raised by the draw circle: {e}')
            return

        # Convert back to ROS Image and publish
        try:
            out_msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Exception raised while image conversion: {e}')
            return
        out_msg.header = self.header  # preserve original header
        self.safe_publish(self.image_pub, out_msg)
        
    def set_marker_center(self, target_marker_id):
        """Determine if the marker with the given target_id is centered in the camera view"""
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, rejected = cv2.aruco.detectMarkers(self.cv_image, aruco_dict, parameters=parameters)
            if ids is None or len(ids) == 0:
                if DEBUG:
                    self.get_logger().warning(f'No cornerns has been detected')
                return False
            else:
                for i, c in enumerate(corners):
                    marker_id = int(ids[i][0])
                    if target_marker_id != marker_id:
                        if DEBUG:
                            self.get_logger().warning(f'Markers id not coincide with the target')
                        continue
                    pts = c.reshape((4, 2))
                    center = pts.mean(axis=0)
                    centered_px = is_centered_pixel(center, self.cv_image.shape, pixel_thresh=9)
                                            
                    if centered_px:
                        self.centered_counts[marker_id] = self.centered_counts.get(marker_id, 0) + 1
                        if DEBUG:
                            self.get_logger().info(f'Frames {self.centered_counts[marker_id]}/{self.REQUIRED_CONSECUTIVE} centered for marker: {marker_id}')     
                    else:
                        self.centered_counts[marker_id] = 0
                        if DEBUG:
                            self.get_logger().warning(f'Framed not centered for marker: {marker_id}')

                    if self.centered_counts[marker_id] >= self.REQUIRED_CONSECUTIVE:
                        if DEBUG:
                            self.get_logger().info(f"Marker {marker_id} CENTERED")
                            cv2.namedWindow(f'marker-{marker_id}-p', cv2.WINDOW_AUTOSIZE)
                            cv2.imshow(f'marker-{marker_id}-p', self.cv_image)
                            cv2.waitKey(20)
                        return True
                return False
        except Exception as e:
            self.get_logger().warning(f'Exception raised by the set marker center: {e}')
            return False
            
    def rotate_360(self, angular_speed):
        """
        First, the robot rotates 360 degrees to detect all the markers around itself. 
        Then it performs individual detections, starting from the marker with the lowest ID. For each marker, the robot rotates clockwise or counterclockwise, 
        depending on the shortest path to align with the marker, stopping when the marker is centered in the camera. Using the OpenCV library, a circle is drawn 
        around the marker. The robot then proced to the next marker, and repeats this process for all remaining markers
        """
        
        self.get_logger().info('Waiting for first odom message...')
        
        # Wait 10 seconds or until self.yaw is inizialized
        wait_count = 0
        while self.yaw is None and wait_count < 200:
            rclpy.spin_once(self, timeout_sec=0.05)
            wait_count += 1
            
        if self.yaw is None:
            self.get_logger().error('No odom received, aborting.')
            return
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self.get_logger().info(f'Rotation begin: speed={angular_speed:.3f} rad/s')
        
        last_yaw = self.yaw
        cumulative_rotation = 0.0
        twist.angular.z = angular_speed
        try:
            # Keep turning until the markers detected are five and he performed a whole 360°
            while (len(self.markers) < self.EXCPECTED_MARKERS) or (cumulative_rotation < (2.0 * math.pi)):
                self.safe_publish(self.vel_pub, twist)
                rclpy.spin_once(self, timeout_sec=0.05)
                if DEBUG:
                    cv2.waitKey(1)
                if self.yaw is not None:
                    delta = normalize_angle(self.yaw - last_yaw)
                    cumulative_rotation += abs(delta)
                    last_yaw = self.yaw
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted during rotation.')
            return

        twist.angular.z = 0.0
        self.safe_publish(self.vel_pub, twist)
        self.get_logger().info('SCANNING COMPLETED!')
        
        try:
            self.destroy_subscription(self.markers_sub)
        except Exception:
            return

        time.sleep(3)
        
        try:
            while len(self.markers) != 0:
                lowest_marker_id = min(self.markers.keys())
                marker_yaw = self.markers[lowest_marker_id][0]
                self.get_logger().info(f'Aligning to marker {lowest_marker_id} (target yaw={math.degrees(marker_yaw):.1f}°)')
                delta = normalize_angle(marker_yaw - self.yaw)
                twist.angular.z = angular_speed if delta > 0.0 else -angular_speed
                while True:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    cv2.waitKey(1)
                    
                    if self.set_marker_center(lowest_marker_id):
                        break  
                    
                    self.safe_publish(self.vel_pub,twist)                   
                twist.angular.z = 0.0
                self.safe_publish(self.vel_pub, twist)
                self.get_logger().info(f'Marker {lowest_marker_id} detected, yaw={math.degrees(self.yaw):.1f}°')
                self.draw_circle(lowest_marker_id)
                self.markers.pop(lowest_marker_id)
                time.sleep(2)
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted during alignment.')
            return
        
        """try:
            threshold = 0.02     # Threshold for aligning the robot with the marker (rads)
            while len(self.markers) != 0:
                lowest_marker_id = min(self.markers.keys())
                marker_yaw = self.markers[lowest_marker_id][0]
                self.get_logger().info(f'Aligning to marker {lowest_marker_id} (target yaw={math.degrees(marker_yaw):.1f}°)')
                while True:
                    rclpy.spin_once(self, timeout_sec=0.01)
                    cv2.waitKey(1)
                    delta = normalize_angle(marker_yaw - self.yaw)
                    if abs(delta) <= threshold:
                        break    
                                    
                    twist.angular.z = angular_speed if delta > 0.0 else -angular_speed
                    self.safe_publish(self.vel_pub, twist)
                    #self.get_logger().info(f'ROBOT {self.yaw} --------- MARKER {marker_yaw}')                    
                twist.angular.z = 0.0
                self.safe_publish(self.vel_pub, twist)
                self.get_logger().info(f'Marker {lowest_marker_id} detected, yaw={math.degrees(self.yaw):.1f}°')
                self.draw_circle(lowest_marker_id)
                self.markers.pop(lowest_marker_id)
                time.sleep(2)
        except KeyboardInterrupt:
            self.get_logger().info('Interrupted during alignment.')
            return
        """
        self.get_logger().info("TASK COMPLETED!")
        
def main(args=None):
    rclpy.init(args=args)
    node = ScanMarkers()
    try:
        node.rotate_360(angular_speed=0.25)
    finally:
        try:
            node.destroy_node()
            print('Press the key "q" while the focus is on one of the pictures to quit the program')
            while True:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'): 
                    break
            cv2.destroyAllWindows()
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
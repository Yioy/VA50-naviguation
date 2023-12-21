#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image as ImageMsg
from fish2bird import to_birdeye
from tf.transformations import quaternion_matrix
import os

def create_transform(translation, rotation):
        """
        Create a 4x4 homogeneous transformation matrix from translation and rotation.

        Parameters:
            translation: List or array-like [x, y, z]
            rotation: List or array-like [x, y, z, w] 

        Returns:
            transform_matrix: 4x4 NumPy array
        """
        translation_matrix = np.eye(4)
        translation_matrix[:3, 3] = [translation.x, translation.y, translation.z]

        rotation_matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])

        transform_matrix = np.dot(translation_matrix, rotation_matrix)
        return transform_matrix
class ImageProcessor:

    def __init__(self):
        rospy.init_node('image_processor_node', anonymous=True)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.image_forward_sub = rospy.Subscriber('/forwardCamera/image_raw', Image, self.forward_image_callback)
        self.image_left_sub = rospy.Subscriber('/forwardLeftCamera/image_raw', Image, self.left_image_callback)
        self.image_right_sub = rospy.Subscriber('/forwardRightCamera/image_raw', Image, self.right_image_callback)


        self.im_left = None
        self.im_center = None
        self.im_right = None

        self.timer = rospy.Timer(rospy.Duration(1.0), self.callback)

    def forward_image_callback(self, msg):
        self.im_center = self.process_image(msg, 'camera_forward_optical_frame')

    def left_image_callback(self, msg):
        self.im_left = self.process_image(msg, 'camera_forward_left_optical_frame')


        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # except Exception as e:
        #     rospy.logerr(f"Error converting image: {e}")
        #     return


        # cv2.imshow("Bird-Eye View", cv_image)
        # cv2.waitKey(1)

    def right_image_callback(self, msg):
        self.im_right = self.process_image(msg, 'camera_forward_right_optical_frame')

    def process_image(self, msg, source_frame):
        try:
            # Get the transformation from camera to base
            camera_to_base_transform = self.tf_buffer.lookup_transform(
                target_frame='base_link',
                source_frame=source_frame,
                time=rospy.Time(0),
                timeout=rospy.Duration(0.1)
            )

            # Access the translation and rotation components
            translation_camera_to_base = camera_to_base_transform.transform.translation
            rotation_camera_to_base = camera_to_base_transform.transform.rotation

            # Create the transformation matrix
            T_camera_to_base = create_transform(translation_camera_to_base, rotation_camera_to_base)

            # Convert sensor_msgs/Image to cv2 image
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                rospy.logerr(f"Error converting image: {e}")
                return


            K = np.array([[1124.66943359375, 0.0, 505.781982421875],
                            [0.0, 1124.6165771484375, 387.8110046386719],
                            [0.0, 0.0, 1.0]])
            


            # Apply bird's eye view transformation
            cv_image_gray = cv2.cvtColor(cv_image.copy(), cv2.COLOR_BGR2GRAY)


            output_image, scale_factor = to_birdeye(cv_image_gray, K,T_camera_to_base, 0.0, [-25, 25], [0, 50], 500)

            output_image = np.array(output_image)

            print("heeeee")

            cv2.imshow("Bird-Eye View", output_image)
            cv2.waitKey(1)

            return 

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error looking up transform: {e}")
            return None
    
    def stitch_images(self, image_left, image_middle, image_right):

        print(image_left.shape)
        print(image_middle.shape)
        print(image_right.shape)

        # Check if all images are not None and have content
        if any(image is None for image in [image_left, image_middle, image_right]):
            print("One or more images are empty")
            return None
        
        # Resize images to a consistent size
        images_resized = [cv2.resize(image, (653, 920)) for image in [image_left, image_middle, image_right]]

        # Check and ensure that images have 3 channels and convert RGBA to BGR
        # images_resized = [cv2.cvtColor(image, cv2.COLOR_RGBA2BGR) if image.shape[2] == 4 else image for image in images_resized]

        return images_resized[1]

        # Create an instance of Stitcher from OpenCV
        stitcher = cv2.Stitcher_create()

        # Perform stitching
        status, stitched = stitcher.stitch(images_resized)

        return stitched

        if status == cv2.Stitcher_OK:
            # Find contours to crop black regions
            gray = cv2.cvtColor(stitched, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Get the largest contour (which should be the panorama)
                max_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(max_contour)
                # Crop the image based on this contour
                stitched = stitched[y:y+h, x:x+w]
                print("Stitching process successful")

            return stitched

        else:
            print("Stitching process failed:", status)
            return None
    
    def callback(self, event=None):
        if self.im_left is None or self.im_center is None or self.im_right is None:
            return

        print(self.im_left.shape)
        print(self.im_center.shape)
        print(self.im_right.shape)

        # im_out = self.stitch_images(self.im_left, self.im_center, self.im_right)

        # print(im_out.shape)
        



if __name__ == '__main__':
    try:
        image_processor = ImageProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
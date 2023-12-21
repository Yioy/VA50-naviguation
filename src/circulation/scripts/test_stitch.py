#!/usr/bin/env python3

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

        self.image_forward_pub = rospy.Publisher('/processed/forward_image', ImageMsg, queue_size=1)
        self.image_left_pub = rospy.Publisher('/processed/left_image', ImageMsg, queue_size=1)
        self.image_right_pub = rospy.Publisher('/processed/right_image', ImageMsg, queue_size=1)

        self.image_forward_sub = rospy.Subscriber('/forwardCamera/image_raw', Image, self.forward_image_callback)
        self.image_left_sub = rospy.Subscriber('/forwardLeftCamera/image_raw', Image, self.left_image_callback)
        self.image_right_sub = rospy.Subscriber('/forwardRightCamera/image_raw', Image, self.right_image_callback)

    def forward_image_callback(self, msg):
        self.process_image(msg, 'camera_forward_optical_frame', self.image_forward_pub)

    def left_image_callback(self, msg):
        self.process_image(msg, 'camera_forward_left_optical_frame', self.image_left_pub)

    def right_image_callback(self, msg):
        self.process_image(msg, 'camera_forward_right_optical_frame', self.image_right_pub)

    def process_image(self, msg, source_frame, publisher):
        try:
            # Get the transformation from camera to base
            camera_to_base_transform = self.tf_buffer.lookup_transform(
                target_frame='base_link',
                source_frame=source_frame,
                time=rospy.Time(0),
                timeout=rospy.Duration(1.0)
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

            # Apply bird's eye view transformation
            output_image, scale_factor = to_birdeye(cv_image, [[1124.66943359375, 0.0, 505.781982421875],[0.0, 1124.6165771484375, 387.8110046386719],[0.0, 0.0, 1.0]],T_camera_to_base, 0.0, [-25, 25], [0, 50], 500)

            # Publish the processed image
            processed_image_msg = self.bridge.cv2_to_imgmsg(output_image, "bgr8")
            publisher.publish(processed_image_msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error looking up transform: {e}")
            return

if __name__ == '__main__':
    try:
        image_processor = ImageProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
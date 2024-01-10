import rospy
from trajectory_extractor.fish2bird_python import Fish2Bird
from trajectory_extractor.MultiCamBirdView import MultiCamBirdViewInitConfig
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
import message_filters
from cv_bridge import CvBridge

import cv2 as cv


class MultiCamBirdViewRosInitializer:
    """Class that creates a MultiCamBirdViewInitConfig object using ros topics"""

    def __init__(self):
        pass

    def create_init_config(self, camera_info_topics, reference_frame_id,
                 x_range, y_range, output_size_y, flip_x=False, flip_y=False):
        """Initialise the MultiCamBirdViewInitConfig class
        :param camera_info_topics: list of camera info topics
        :param camera_frame_ids: list of camera frame ids
        :param reference_frame_id: reference frame id
        :param x_range: range [min, max] of x values in the bird view in meters
        :param y_range: range [min, max] of y values in the bird view in meters
        :param output_size_y: size of the bird view image in pixels
        :param flip_x: flip the x axis of the bird view image
        :param flip_y: flip the y axis of the bird view image
        """
        self.camera_info_topics = camera_info_topics
        self.reference_frame_id = reference_frame_id

        self.camera_infos = [None] * len(camera_info_topics)

        # Subsrcibe to camera info topics. Create a callback for each camera
        self.camera_info_subscribers = []
        for i in range(len(camera_info_topics)):
            self.camera_info_subscribers.append(rospy.Subscriber(camera_info_topics[i], CameraInfo, self.camera_info_callback, i))
        
        # Wait for camera info topics to be received
        rospy.loginfo("Waiting for camera info topics...")
        while None in self.camera_infos and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Camera info messages received")


        # Get transforms from tf
        self.transforms = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        camera_frame_ids = [camera_info.header.frame_id for camera_info in self.camera_infos]

        rospy.loginfo("Waiting for transforms...")
        while self.transforms is None and not rospy.is_shutdown():
            self.transforms = self.get_transforms(camera_frame_ids, self.reference_frame_id)
            rospy.sleep(0.1)
        rospy.loginfo("Transforms received")

        # create Ks and Ds
        Ks = []
        Ds = []
        for i in range(len(self.camera_infos)):
            Ks.append(self.camera_infos[i].K)
            Ds.append(self.camera_infos[i].D)
        
        # Create the config
        config = MultiCamBirdViewInitConfig(
            self.camera_infos[0].height,
            self.camera_infos[0].width,
            Ks,
            Ds,
            self.transforms,
            x_range,
            y_range,
            output_size_y,
            flip_x=flip_x,
            flip_y=flip_y
        )

        return config


    def camera_info_callback(self, msg, camera_index):
        """Callback for camera info topic
        :param msg: camera info message
        :param camera_index: index of the camera
        """

        # Check if model is Mei model
        if msg.distortion_model.lower() != "mei":
            rospy.logerr_once(f"Distortion model {msg.distortion_model} not supported. Only mei distortion model is supported")
            return

        # Unsubscribe from camera info topic
        self.camera_info_subscribers[camera_index].unregister()

        # Add camera info to list
        self.camera_infos[camera_index] = msg

    
    # def camera_callback(self, *args):
    #     """Callback for camera topics
    #     :param args: list of camera images
    #     """

    #     # Convert camera images to cv images
    #     images = []
    #     try:
    #         images = [self.bridge.imgmsg_to_cv2(image, desired_encoding="mono8") for image in args]
    #     except Exception as e:
    #         rospy.logerr(f"Error converting image: {e}")
    #         return
    #     # Process the images
    #     self.process_images(images)


    def get_transforms(self, from_frames, to_frame):
        """Get the transformation matrices from the camera frames to the reference frame
        :param from_frames: list of camera frame ids
        :param to_frame: reference frame id
        :return: list of transformation matrices
        """
        transforms = []
        for i in range(len(from_frames)):
            success, T = self.get_transform(from_frames[i], to_frame)
            if success:
                transforms.append(T)
            else:
                rospy.logwarn_once(f"Could not get transform from {from_frames[i]} to {to_frame}. Retrying...")
                return None
        return transforms
        


    def get_transform(self, from_frame, to_frame):
        try:
            # Get the transformation from camera to base
            camera_to_base_transform = self.tf_buffer.lookup_transform(
                target_frame=to_frame, # base_link
                source_frame=from_frame,
                time=rospy.Time(0),
                timeout=rospy.Duration(0.1)
            )

            # Access the translation and rotation components
            translation_camera_to_base = camera_to_base_transform.transform.translation
            rotation_camera_to_base = camera_to_base_transform.transform.rotation

            rospy.loginfo(f"Got transform from {from_frame} to {to_frame}")

            # Create the transformation matrix
            T = self.create_transform(translation_camera_to_base, rotation_camera_to_base)
            return True, T
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error looking up transform: {e}")
            return False, None



    def create_transform(self, translation, rotation):

        r = R.from_quat([rotation.x, rotation.y, rotation.z, rotation.w])
        T = np.vstack((
            np.hstack((r.as_matrix(), 
                        np.array([[translation.x], [translation.y], [translation.z]]))),
            np.array([0, 0, 0, 1])))
        
        return T
        
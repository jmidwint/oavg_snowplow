#!/usr/bin/env python

import rospy
from oavg_msgs.msg import ObstacleStatus
import jetson.utils
import jetson.inference
from os import path

class ObstacleDetection:
    """
        Uses the camera and a machine learning model to detect pylons obstructing the robot's path.
        Publishes to the /oavg_obstacle topic indicating if the path is blocked or not.
    """

    def __init__(self):
        """Initialize topic and configuration parameters."""
        self.pub = rospy.Publisher("/oavg_obstacle", ObstacleStatus, queue_size=1)
        self.initialize_default_values()
        self.initialize_values()

    def initialize_default_values(self):
        """Provide reasonable defaults for most configuration values."""
        self.default_oavg_camera = "csi://0"
        self.default_oavg_camera_flip = ""
        self.default_oavg_camera_frames_per_second = 2
        self.default_oavg_obstacle_threshold = 0.6

    def initialize_values(self):
        """Retrive configuration values from the ROS Parameter Server."""
        self.oavg_camera = rospy.get_param("/oavg_camera", self.default_oavg_camera)
        self.oavg_camera_flip = rospy.get_param("/oavg_camera_flip", self.default_oavg_camera_flip)
        self.oavg_camera_frames_per_second = rospy.get_param("/oavg_camera_frames_per_second", self.default_oavg_camera_frames_per_second)
        self.oavg_obstacle_threshold = rospy.get_param("/oavg_obstacle_threshold", self.default_oavg_obstacle_threshold)
        try:
            self.oavg_obstacle_detection_model_file = rospy.get_param("/oavg_obstacle_detection_model_file")
            self.oavg_obstacle_detection_label_file = rospy.get_param("/oavg_obstacle_detection_label_file")

            if (not path.isfile(self.oavg_obstacle_detection_model_file)) or (not path.isfile(self.oavg_obstacle_detection_label_file)):
                err = "Obstacle detection ML model file not found. Run 'catkin_make download_extra_data' to retrieve file."
                rospy.logerr(err)
                raise rospy.exceptions.ROSInitException(err)
        except KeyError as e:
            err = "Obstacle detection ML model params not found. Ensure '/oavg_obstacle_detection_model_file' and '/oavg_obstacle_detection_label_file' are set."
            rospy.logerr(err)
            raise rospy.exceptions.ROSInitException(err)

    def run(self):
        """Use the camera and a machine learning model to continually determine if the robot's path is blocked."""
        model = "--model={}".format(self.oavg_obstacle_detection_model_file)
        label = "--labels={}".format(self.oavg_obstacle_detection_label_file)
        net = jetson.inference.imageNet("resnet-18",[model, label, "--input_blob=input_0", "--output_blob=output_0"])

        rate = rospy.Rate(self.oavg_camera_frames_per_second)

        camera = jetson.utils.videoSource(self.oavg_camera)
        
        try:
            camera.Open()

            while camera.IsStreaming() and not rospy.is_shutdown():
                img = camera.Capture()
                detections, _ = net.Classify(img)
                class_description = net.GetClassDesc(detections)

                status = ObstacleStatus()
                status.blocked = False
                if class_description == "blocked":
                    status.blocked = True
                self.pub.publish(status)

                rate.sleep()
        except Exception as e:
            rospy.logerr("Error during obstacle detection: {}".format(e))
        finally:
            camera.Close()


if __name__ == '__main__':
    rospy.init_node("oavg_obstacle_detection")

    obstacleDetection = ObstacleDetection()
    obstacleDetection.run()

    rospy.loginfo("Obstacle detection node has ended")

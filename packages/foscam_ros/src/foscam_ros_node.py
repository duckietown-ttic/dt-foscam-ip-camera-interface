#!/usr/bin/env python
import os
import time
import thread
import yaml

# ROS libs
import rospy
from sensor_msgs.msg import CompressedImage, CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

# Foscam libs
from libpyfoscam import FoscamCamera, FOSCAM_SUCCESS, FoscamError

# Duckietown libs
from duckietown import DTROS


class FoscamCameraNode(DTROS):
    """
    This ROS node takes a stream of images from a Foscam IP camera using the HTTP CGI API
    provided by such cameras and turns it into a stream of CompressedImage ROS messages.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use
        camera_name (:obj:`str`): a unique name for the camera

    Configuration:
        ~ip (:obj:`float`): The camera IP
        ~port (:obj:`float`): The TCP port the HTTP API of the camera is available on
        ~username (:obj:`float`): The username to login into the camera
        ~password (:obj:`float`): The password to login into the camera
        ~framerate (:obj:`float`): The maximum camera image acquisition framerate, defaults to the max supported by the camera

    Publisher:
        ~image/compressed (:obj:`CompressedImage`): The acquired camera images

    Service:
        ~set_camera_info:
            Saves a provided camera info to `/data/config/calibrations/camera_intrinsic/(camera_name).yaml`.
            input:
                camera_info (`CameraInfo`): The camera information to save
            outputs:
                success (`bool`): `True` if the call succeeded
                status_message (`str`): Used to give details about success
    """

    def __init__(self, node_name, camera_name):
        # Initialize the DTROS parent class
        super(FoscamCameraNode, self).__init__(node_name=node_name)

        # Add the node parameters to the parameters dictionary and load their default values
        self.camera_name = camera_name
        self.parameters['~ip'] = None
        self.parameters['~port'] = None
        self.parameters['~username'] = None
        self.parameters['~password'] = None
        self.parameters['~framerate'] = None
        self.updateParameters()

        # Setup FoscamCamera
        self.image_msg = CompressedImage()
        self.initializeCamera()

        # For intrinsic calibration
        self.cali_file_folder = '/data/config/calibrations/camera_intrinsic'
        self.frame_id = rospy.get_namespace().strip('/') + '/camera_optical_frame'
        self.cali_file = "%s/%s.yaml" % (self.cali_file_folder, self.camera_name)

        # Locate calibration yaml file or shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            msg = "Can't find calibration file: %s.\n Aborting!" % self.cali_file
            rospy.signal_shutdown(msg)

        # Load the calibration file
        self.camera_info = self.loadCameraInfo(self.cali_file)
        self.log("Using calibration file: %s" % self.cali_file)

        # Setup publishers
        self.has_published = False
        self.pub_img = self.publisher("~image/compressed", CompressedImage, queue_size=1)
        self.pub_camera_info = self.publisher("~camera_info", CameraInfo, queue_size=1)

        # Setup service (for camera_calibration)
        self.srv_set_camera_info = rospy.Service("~set_camera_info",
                                                 SetCameraInfo,
                                                 self.cbSrvSetCameraInfo)
        # ---
        self.log("Initialized.")

    def initializeCamera(self):
        self.camera = FoscamCamera(
            self.parameters['~ip'],
            self.parameters['~port'],
            self.parameters['~username'],
            self.parameters['~password'],
            daemon=False
        )

    def startCapturing(self):
        """Initialize and closes image stream.
            Begin the camera capturing. When the node shutdowns, closes the
            image stream. If it detects a parameter change, will update the parameters and
            restart the image capturing.
        """
        self.log("Start capturing.")
        stime = time.time()
        wtime = 1.0 / float(self.parameters['~framerate'])
        while not (self.is_shutdown or rospy.is_shutdown()):
            # adjust framerate
            delta = time.time() - stime
            if delta < wtime:
                time.sleep(wtime - delta)
            stime = time.time()
            # try fetching a new frame from the camera
            try:
                self.grabAndPublish()
            except FoscamError as e:
                self.log('Error fetching frame from the camera, the API returned:\n%s' % e, 'warn')
            # ---
            if self.parametersChanged:
                # update parameters
                self.parametersChanged = False
                self.initializeCamera()
                self.log("Parameters updated.")
        # ---
        self.log("Capture Ended.")

    def grabAndPublish(self):
        """Captures a frame from stream and publishes it.
            If the stream is stable (no shutdown requests),
            grabs a frame, creates the image message and publishes it.
        """
        # get time
        stamp = rospy.Time.now()

        # fetch frame
        code, data = self.camera.snap_picture()
        if code != FOSCAM_SUCCESS:
            raise FoscamError(code=code)

        # generate and publish the compressed image
        image_msg = CompressedImage()
        image_msg.format = "jpeg"
        image_msg.data = data
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id
        self.pub_img.publish(image_msg)

        # Publish the CameraInfo message
        self.camera_info.header.stamp = stamp
        self.pub_camera_info.publish(self.camera_info)

        if not self.has_published:
            self.log("Published the first image.")
            self.has_published = True

    def cbSrvSetCameraInfo(self, req):
        self.log("[cbSrvSetCameraInfo] Callback!")
        response = SetCameraInfoResponse()
        response.success = self.saveCameraInfo(req.camera_info, self.cali_file)
        response.status_message = "Write to %s" % self.cali_file
        return response

    def loadCameraInfo(self, filename):
        """Loads the camera calibration files.
        Loads the intrinsic and extrinsic camera matrices.
        Args:
            filename (:obj:`str`): filename of calibration files.
        Returns:
            :obj:`CameraInfo`: a CameraInfo message object
        """
        stream = file(filename, 'r')
        calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info

    def saveCameraInfo(self, camera_info_msg, filename):
        """Saves intrinsic calibration to file.
            Args:
                camera_info_msg (:obj:`CameraInfo`): Camera Info containg calibration
                filename (:obj:`str`): filename where to save calibration
        """
        # Convert camera_info_msg and save to a yaml file
        self.log("[saveCameraInfo] filename: %s" % (filename))

        # Converted from camera_info_manager.py
        calib = {'image_width': camera_info_msg.width,
                 'image_height': camera_info_msg.height,
                 'camera_name': rospy.get_name().strip("/"),  # TODO check this
                 'distortion_model': camera_info_msg.distortion_model,
                 'distortion_coefficients': {'data': camera_info_msg.D,
                                             'rows': 1,
                                             'cols': 5},
                 'camera_matrix': {'data': camera_info_msg.K,
                                   'rows': 3,
                                   'cols': 3},
                 'rectification_matrix': {'data': camera_info_msg.R,
                                          'rows': 3,
                                          'cols': 3},
                 'projection_matrix': {'data': camera_info_msg.P,
                                       'rows': 3,
                                       'cols': 4}}

        self.log("[saveCameraInfo] calib %s" % (calib))

        try:
            f = open(filename, 'wt')
            yaml.safe_dump(calib, f)
            return True
        except IOError:
            return False


if __name__ == '__main__':
    # Initialize the node
    camera_node = FoscamCameraNode(node_name='camera', camera_name='foscam_r2')
    # Start the image capturing in a separate thread
    thread.start_new_thread(camera_node.startCapturing, ())
    # Keep it spinning to keep the node alive
    rospy.spin()

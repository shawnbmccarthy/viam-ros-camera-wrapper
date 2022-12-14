import cv2
import PIL
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from viam.components.camera import Camera, DistortionParameters, IntrinsicParameters


class RosCamera(Camera):
    def __init__(self, name, topic):
        """
        initialize our camera, ros node and ros subscriber

        :param name:
        :param topic:
        """
        rospy.init_node('viam_camera_stream', anonymous=True)
        rospy.Subscriber(topic, Image, self.callback)
        self.props = Camera.Properties(
            supports_pcd=False,
            distortion_parameters=DistortionParameters(),
            intrinsic_parameters=IntrinsicParameters()
        )
        self.image = None
        super().__init__(name)

    def callback(self, data):
        """
        the callback function used to pic up images off the queue, we ill then convert to a PNG image

        :param data:
        :return:
        """
        br = CvBridge()
        tmp = br.imgmsg_to_cv2(data)
        tmp = cv2.cvtColor(tmp, cv2.COLOR_BGR2RGB)
        self.image = PIL.Image.fromarray(tmp)

    async def get_image(self, mime_type='', **kwargs):
        """
        get the next image, since the callback is updating the image from the ros topic we simply return
        the updated image

        return the image
        :param mime_type:
        :param kwargs:
        :return:
        """
        return self.image

    async def get_point_cloud(self, **kwargs):
        """
        camera does not support point cloud
        :param kwargs:
        :return:
        """
        return None

    async def get_properties(self, **kwargs):
        """
        return supported properties

        :param kwargs:
        :return:
        """
        return self.props

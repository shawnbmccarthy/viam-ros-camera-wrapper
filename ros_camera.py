import cv2
import PIL
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from typing import ClassVar, Mapping, Sequence, Any, Dict, Optional, cast
from typing_extensions import Self
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.types import Model, ModelFamily
from viam.components.camera import Camera, DistortionParameters, IntrinsicParameters, RawImage


class RosCamers(Base, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'ros'), 'roscamera')

    topic: str
    props: Camera.Properties
    image: PIL.Image

    @classmethod
    def new_camera(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        camera = cls(RosCamera(config.name))
        camera.reconfigure(config, dependencies)
        return camera 

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        topic = config.attributes.fields['topic'].string_value
        if topic == '':
            raise Exception("A ros topic is required for roscamera component.")
        return [topic]

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> None:
        topic = config.attributes.fields['topic'].string_value
        self.topic = topic
        rospy.init_node('viam_camera_stream', anonymous=True)
        rospy.Subscriber(topic, Image, self.callback)
        self.props = Camera.Properties(
            supports_pcd = False,
            distortion_parameters = DistortionParameters(),
            intrinsic_parameters = IntrinsicParameters()
        )

    def image_callback(self, data) -> None:
        br = CvBridge()
        tmp = br.image_to_cv2(data)
        tmp = cv2.cvtColor(tmp, cv2.COLOR_BGR2RGB)
        self.image = PIL.Image.fromarray(tmp)

    async def get_image(
        self,
        mime_type: str = '',
        timeout: Optional[float] = None,
        **kwargs
    ) -> Union[PIL.Image.Image, RawImage]:
        return self.image

    async def get_point_cloud(self, *, timeout: Optional[float] = None, **kwargs) -> Tuple[bytes, str]:
        return None

    async def get_properties(*, timeout: Optional[float] = None, **kwargs) -> Camera.Properties:
        return self.props

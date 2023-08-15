import rospy

from typing import Any, ClassVar, Mapping, Optional, Sequence, Tuple, Union
from typing_extensions import Self
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.components.sensor import Sensor


class RosSensor(Sensor, Reconfigurable):
    """
    Need to create attributes for messages
    """
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'ros'), 'sensor')

    topic: str

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        sensor = cls(RosSensor(config.name))
        sensor.reconfigure(config, dependencies)
        return sensor

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        topic = config.attributes.fields['topic'].string_value
        if topic == '':
            raise Exception("A components topic is required for ros sensor component.")
        return [topic]

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> None:
        """
        TODO: Need to add message to subscribe to here
        """
        topic = config.attributes.fields['topic'].string_value
        self.topic = topic
        rospy.Subscriber(topic, None, self.callback)


    def callback(self, data) -> None:
        """
        TODO: implement msg processing
        """
        return None

    async def get_readings(
        self,
        *,
        extra: Optional[Mapping[str, Any]],
        timeout: Optional[float]=None,
        **kwargs
    ) -> Mapping[str, Any]:
        """
        TODO: return sensor msg here
        """
        return {
            "key": "must implement"
        }


Registry.register_resource_creator(
    Sensor.SUBTYPE,
    RosSensor.MODEL,
    ResourceCreatorRegistration(RosSensor.new, RosSensor.validate_config)
)

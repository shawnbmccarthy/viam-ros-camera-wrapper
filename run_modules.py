import asyncio
import logging
import sys
import rospy
from components import RosCamera, RosSensor
from viam.components.camera import Camera
from viam.components.sensor import Sensor
from viam.module.module import Module
from viam.logging import getLogger

logger = getLogger(__name__)


def rospy_shutdown_hook():
    logger.info('shutting down rospy')

async def main(addr: str) -> None:
    module = Module(addr)
    module.add_model_from_registry(Camera.SUBTYPE, RosCamera.MODEL)
    module.add_model_from_registry(Sensor.SUBTYPE, RosSensor.MODEL)
    await module.start()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise Exception('need socket path as argument')
    rospy.init('viam_ros_node', anonymous=False)
    rospy.on_shutdown(rospy_shutdown_hook)
    asyncio.run(main(sys.argv[1]))
    rospy.signal_shutdown('module shutdown')


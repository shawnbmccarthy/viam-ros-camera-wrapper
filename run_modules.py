import asyncio
import sys

from ros_camera import RosCamera
from viam.components.camera import Camera
from viam.module.module import Module


async def main(addr: str) -> None:
    module = Module(addr)
    module.add_from_registry(Camera.SUBTYPE, RosCamera.MODEL)
    await module.start()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise Exception('need socket path as argument')
    asyncio.run(main(sys.argv[1]))

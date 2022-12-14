import argparse
import asyncio
import logging

from viam.rpc.server import Server

from ros_camera import RosCamera


def parse_args():
    """
    simple argument parser

    :return:
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--host',
        '-n',
        dest='host',
        help='hostname/ip rpc server will bind to',
        required=False,
        type=str,
        action='store',
        default='localhost'
    )
    parser.add_argument(
        '--port',
        '-p',
        dest='port',
        help='port number to store',
        required=False,
        type=int,
        action='store',
        default=9090
    )
    parser.add_argument(
        '--log',
        '-l',
        dest='log',
        help='log level to use',
        required=False,
        type=str,
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'FATAL'],
        default='WARNING'
    )
    parser.add_argument(
        '--topic',
        '-t',
        dest='topic',
        help='ros camera topic',
        required=False,
        type=str,
        default='camera/image_raw'
    )
    c_args = parser.parse_args()
    c_args.log = getattr(logging, c_args.log)
    return c_args


async def main(host, port, log_level, topic):
    """
    start an RPC server to host custom remote components

    :param host:
    :param port:
    :param log_level:
    :return:
    """
    srv = Server([RosCamera('ros-wrapped-camera', topic=topic)])
    await srv.serve(host=host, port=port, log_level=log_level)


if __name__ == '__main__':
    try:
        args = parse_args()
        asyncio.run(main(args.host, args.port, args.log))
    except Exception as e:
        logging.error(f'failed to start server: {e}')

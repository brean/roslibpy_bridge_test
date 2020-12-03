import os
import time
import roslibpy
import logging


logger = logging.getLogger('ros_bridge')

SERVICE_FROM_ROBOT = [
    (
        '/my_service',
        'custom_service/CustomService'
    )
]


class AsyncServiceWorkaround(roslibpy.Service):
    def _service_response_handler(self, request):
        response = roslibpy.ServiceResponse()
        self._service_callback(request, response)

    def after_service_response(self, request, response, success):
        call = roslibpy.Message({'op': 'service_response',
                        'service': self.name,
                        'values': dict(response),
                        'result': success
                        })

        if 'id' in request:
            call['id'] = request['id']

        self.ros.send_on_ready(call)


class RosCon:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.client = None
        while not self.connect():
            time.sleep(1)

    def connect(self):
        """Connect to ROS-Bridge suite."""
        logger.info(f'connecting to ROS WebSuite {self.host}:{self.port}...')
        if not self.client:
            self.client = roslibpy.Ros(host=self.host, port=self.port)
        if self.client.is_connected:
            logger.info(
                f'already connected to {self.host}:{self.port}, ignoring'
                'connect-call')
            return True
        try:
            self.client.run()
            logger.info(f'connected to {self.host}:{self.port}!')
            return True
        except Exception as e:
            if str(e) == 'Failed to connect to ROS':
                logger.warning('Connection error, is roscore running?')
            else:
                # something else besides a connection error
                # might be sth. serious so we rethrow it to exit!
                raise e
        return False



def setup_logging(logger):
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter(
        '[%(levelname)s] %(name)s: %(message)s'))
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)


def main():
    cloud = RosCon(
        host=os.environ['ROS_SERVICE_HOST'],
        port=int(os.environ['ROS_SERVICE_PORT'])).client
    robot = RosCon(
        host=os.environ['ROS_ROBOT_HOST'],
        port=int(os.environ['ROS_ROBOT_PORT'])).client

    for name, service_type in SERVICE_FROM_ROBOT:
        cloud_service = roslibpy.Service(cloud, name, service_type)
        robot_service = AsyncServiceWorkaround(robot, name, service_type)

        def handler(request, response):
            cloud_request = roslibpy.ServiceRequest()
            def cloud_cb(result):
                for key, value in result.items():
                    response[key] = value
                robot_service.after_service_response(request, response, True)
            cloud_service.call(cloud_request, callback=cloud_cb)

        robot_service.advertise(handler)
    
    while True:
        time.sleep(1)


if __name__ == '__main__':
    setup_logging(logger)
    main()
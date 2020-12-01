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

    def create_service(self, name, service_type):
        logger.info(f'create service {name} for {self.host}:{self.port}')
        service = roslibpy.Service(self.client, name, service_type)
        return service

    def get_services_for_type(self, service_type, **kwargs):
        return self.client.get_services_for_type(service_type, **kwargs)

    def errfunc(self, err):
        logger.error(f'error occured while calling a service: {err}')

    def call_service(self, service, name, service_type):
        request = roslibpy.ServiceRequest()
        logger.info(f'call service {name} on {self.host}:{self.port}')
        result = service.call(request, errback=self.errfunc)
        logger.info(f'service response for {name} at '
            f'{self.host}:{self.port}: {result}')
        return result


def setup_logging(logger):
    handler = logging.StreamHandler()
    handler.setFormatter(logging.Formatter(
        '[%(levelname)s] %(name)s: %(message)s'))
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)


def main():
    service = RosCon(
        host=os.environ['ROS_SERVICE_HOST'],
        port=int(os.environ['ROS_SERVICE_PORT']))
    robot = RosCon(
        host=os.environ['ROS_ROBOT_HOST'],
        port=int(os.environ['ROS_ROBOT_PORT']))

    for name, service_type in SERVICE_FROM_ROBOT:
        cloud_service = service.create_service(name, service_type)
        robot_service = robot.create_service(name, service_type)
        def handler(request, response):
            # forward response from cloud service to robot
            result = service.call_service(
                cloud_service, name, service_type).data
            
            logger.info(f'Result from service: {result}')
            for key, value in result.items():
                response[key] = value
            return True
        robot_service.advertise(handler)
    
    while True:
        time.sleep(1)


if __name__ == '__main__':
    setup_logging(logger)
    main()
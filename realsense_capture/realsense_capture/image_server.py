import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from realsense_interface_msg.srv import TakeImage
from sensor_msgs.msg import Image
import functools
import time

class ImageServer(Node):

    def __init__(self):
        super().__init__('image_server')

        # for multi-threaded callback
        multithread_group = ReentrantCallbackGroup()

        # parameter handling
        self.declare_parameter(name = 'camera_namespace', value = 'xArm6')
        self.declare_parameter(name = 'camera_name', value = 'D405')

        self.camere_namespace = self.get_parameter('camera_namespace').get_parameter_value().string_value
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value

        # Subscribe to rgb
        self.rgb_sub = self.create_subscription(
            msg_type = Image, 
            topic = f'/{self.camere_namespace}/{self.camera_name}/color/image_rect_raw', 
            callback = functools.partial(self.sub_image_callback, 'rgb'), 
            qos_profile = 10,
            callback_group = multithread_group)
        self.rgb_buffer = None

        # Subscribe to depth
        self.depth_sub = self.create_subscription(
            msg_type = Image, 
            topic = f'/{self.camere_namespace}/{self.camera_name}/aligned_depth_to_color/image_raw', 
            callback = functools.partial(self.sub_image_callback, 'depth'), 
            qos_profile = 10,
            callback_group = multithread_group)
        self.depth_buffer = None

        # Stores image frame
        self._cam2buffer = {
            'rgb': self.rgb_buffer,
            'depth': self.depth_buffer,
        }

        # Binary flag that specifies whether to record the buffer
        self._cam2capture = {cam: False for cam in self._cam2buffer.keys()}
        
        # Service for rgb
        self.rgb_service = self.create_service(
            srv_type=TakeImage,
            srv_name='/realsense_capture/get_rgb',
            callback=self.get_image_callback, 
            callback_group=multithread_group)

        # Service for depth
        self.depth_service = self.create_service(
            srv_type=TakeImage,
            srv_name='/realsense_capture/get_depth',
            callback=self.get_image_callback,
            callback_group=multithread_group)

    def sub_image_callback(self, cam, frame):
        """ 
        Callback for image subscription.
        Stores the frame only if `capture` is True
        """
        if cam not in self._cam2buffer:
            raise KeyError(f'Unknown camera {cam}')

        if self._cam2capture[cam]:
            self._cam2buffer[cam] = frame
        else:
            self._cam2buffer[cam] = None

    def get_image_callback(self, request, response):
        '''
        Callback for the service that returns the image.
        '''
        self.get_logger().info(f'Request for {request.modality_name.data} image received')
        cam = request.modality_name.data
        if cam not in self._cam2buffer:
            raise KeyError(f'Unknown camera {cam}')

        # Set capture flag to True
        now = time.perf_counter()
        self._cam2capture[cam] = True
        while self._cam2buffer[cam] is None:
            continue
        elapsed = time.perf_counter() - now
        self.get_logger().info(f'Took {elapsed:.3f} seconds to get {cam} image!')

        response.frame = self._cam2buffer[cam]
        
        # Reset the flags
        # self._cam2capture = {cam: False for cam in self._cam2buffer.keys()}
        self._cam2capture[cam] = False
        return response



def main(args=None):
    rclpy.init(args=args)

    server = ImageServer()

    executor = MultiThreadedExecutor()
    executor.add_node(server)

    executor.spin()

    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

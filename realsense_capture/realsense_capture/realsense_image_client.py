from realsense_interface_msg.srv import TakeImage

from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image

from queue import Queue

import os
import base64
import numpy as np
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node



class ImageClient(Node):

    def __init__(self):
        super().__init__('image_client') # type: ignore


        ############################ Miscanellous Setup #######################################
        self.cvbridge = CvBridge() # for converting ROS images to OpenCV images

        self._debounce_setup() # for debouncing the capture button
        self.shutter = False # when this is true, the client will issue a request to the server to capture images

        ############################ Launch Parameters ########################################
        # parameter handling
        self.declare_parameter(name = 'save_folder', value = '/home/irving/Desktop')
        self.save_folder = self.get_parameter('save_folder').get_parameter_value().string_value

        ############################ Client Setup #############################################
        # rgb client
        self.rgb_cli = self.create_client(
            srv_type=TakeImage, 
            srv_name='/realsense_capture/get_rgb')
        while not self.rgb_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('RGB service not available, waiting again...')
        
        # depth client
        self.depth_cli = self.create_client(
            srv_type=TakeImage, 
            srv_name='/realsense_capture/get_depth')
        while not self.depth_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Depth service not available, waiting again...')
        
        self.rgb_req = TakeImage.Request()
        self.rgb_req.modality_name.data = 'rgb'

        self.depth_req = TakeImage.Request()
        self.depth_req.modality_name.data = 'depth'

        ############################ Publisher Setup ##########################################
        self.rgb_publisher = self.create_publisher(
            msg_type=Image, 
            topic='/realsense_capture/captured_rgb_image', 
            qos_profile=10)
        
        self.depth_publisher = self.create_publisher(
            msg_type=Image, 
            topic='/realsense_capture/captured_depth_image', 
            qos_profile=10)
        
        ############################ Subscriber Setup #########################################
        # subscribe to joy topic to read joystick button press
        self.joy_sub = self.create_subscription(
            msg_type=Joy, 
            topic='/joy', 
            callback=self.joy_callback, 
            qos_profile=10)

    def joy_callback(self, msg):
        
        old_value = self.debounce_buffer.get() # pop the oldest read
        self.debounce_buffer.put(msg.buttons[8]) # push the newest read
        if old_value == 0 and msg.buttons[8] == 1: 
            self.shutter = True # rising edge detected

    def postprocess(self, img, modality):
        '''
        img: sensor_msgs.msg.Image
        modality: str
        Use cv_bridge to convert ROS image to OpenCV image and save it to disk
        '''
        # this is in RGB
        if modality == 'rgb':
            self.rgb_publisher.publish(img)
        elif modality == 'depth':
            self.depth_publisher.publish(img)

        encoded_img = self.cvbridge.imgmsg_to_cv2(img_msg=img, 
                                                  desired_encoding='passthrough')
        # color the log message
        color_start = '\033[94m'
        color_reset = '\033[0m'
        
        if self.save_folder != '':
            cv2.imwrite(os.path.join(self.save_folder, f'{modality}_{img.header.stamp.sec}.png'), cv2.cvtColor(encoded_img, cv2.COLOR_RGB2BGR))
            self.get_logger().info(f'{color_start}{modality} image saved{color_reset}')
        else:
            self.get_logger().info(f'{color_start}{modality} image captured, not saved{color_reset}')
        return encoded_img
    
    def depth_postprocess(self, img):
        '''
        I didn't use this. Not sure if normalizing is necessary anymore after I started to use cv_bridge. But keep it just in case
        '''
        base64_bytes = img.data.encode('ascii')
        image_bytes = base64.b64decode(base64_bytes)
        np_arr = np.frombuffer(image_bytes, dtype=np.uint16)

        if img.encoding == '16UC1':
            np_arr = np_arr.reshape((img.height, img.width))
            img = np_arr.copy()
            img[img > 1000] = 0
            normalized_img = (img - img.min()) / (img.max() - img.min())
            normalized_img = 255 * normalized_img
            # (Optional) crop propotion that is lost to aligning detph to color
            # np_arr = np_arr[:, 80:]
        else:
            raise NotImplementedError(f'Cannot deal with encoding {img["encoding"]} that is not "16UC1"')
        cv2.imwrite('/home/irving/Desktop/depth.png', normalized_img)

    def _debounce_setup(self):
        '''
        As in any embedded system, we need to debounce the capture button.
        While we human think we press the button once, the computer actually consider the button pressed all the time during the duration of the press,
        because the polling rate is much faster than the human reaction time. 
        
        This function sets up a buffer to store the last value of the button press so that we can detect the rising edge.
        '''

        self.debounce_buffer = Queue(maxsize=1)
        self.debounce_buffer.put(0) # when nothing is pressed, the value is 0

def main(args=None):
    rclpy.init(args=args)

    client = ImageClient()
    while rclpy.ok():
        if client.shutter: # shutter down
            
            # Not exactly the most performant way to do things, because we are sorta making calls sequentially
            # But it's good enough for now

            # send request to server to capture images
            rgb_future = client.rgb_cli.call_async(client.rgb_req)

            # immediately shutter up to debounce, so we don't caputre multiple images
            client.shutter = False
            
            client.get_logger().info('Request to capture rgb image sent...')
            depth_future = client.depth_cli.call_async(client.depth_req)
            client.get_logger().info('Request to capture depth image sent...')
            
            # wait for the server to capture images
            rclpy.spin_until_future_complete(client, rgb_future)
            rclpy.spin_until_future_complete(client, depth_future)

            # get the images from the server
            client.get_logger().info('Images Acquired...')
            rgb_response = rgb_future.result()
            depth_response = depth_future.result()

            # postprocess the images
            client.postprocess(rgb_response.frame, modality='rgb')
            client.postprocess(depth_response.frame, modality='depth')

            

        rclpy.spin_once(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
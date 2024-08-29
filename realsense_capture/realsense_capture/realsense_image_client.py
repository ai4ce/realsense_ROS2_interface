from realsense_interface_msg.srv import TakeImage

from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from queue import Queue

import os
import base64
import numpy as np
import yaml
import json
from scipy.spatial.transform import Rotation

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

class ImageClient(Node):

    def __init__(self):
        super().__init__('image_client') # type: ignore

        ############################ Launch Parameters ########################################
        # parameter handling
        self.declare_parameter(name = 'save_folder', value = '/home/irving/Desktop')
        self.save_folder = self.get_parameter('save_folder').get_parameter_value().string_value
        self.declare_parameter(name = 'calibration_path', value = '')
        calibration_path = self.get_parameter('calibration_path').get_parameter_value().string_value
        self.declare_parameter(name = 'save_json_path', value = '')
        self.json_path = self.get_parameter('save_json_path').get_parameter_value().string_value

        ############################ Miscanellous Setup #######################################
        self.cvbridge = CvBridge() # for converting ROS images to OpenCV images

        self._debounce_setup() # for debouncing the capture button
        self.shutter = False # when this is true, the client will issue a request to the server to capture images
        
        # create a folder to save the images
        # save_folder/camera/rgb + save_folder/camera/depth
        os.makedirs(self.save_folder, exist_ok=True)
        self.save_folder = os.path.join(self.save_folder, 'camera')
        os.makedirs(self.save_folder, exist_ok=True)
        self.rgb_save_folder = os.path.join(self.save_folder, 'rgb')
        os.makedirs(self.rgb_save_folder, exist_ok=True)
        self.depth_save_folder = os.path.join(self.save_folder, 'depth')
        os.makedirs(self.depth_save_folder, exist_ok=True)

        self.rgb_count = 0
        self.depth_count = 0

        ############################ JSON Setup ###############################################
        if self.json_path != '':
            self.json_dict = {}
            self._json_setup(calibration_path)

        ############################ TF Setup #################################################
        # buffer to hold the transform in a cache
        self.tf_buffer = Buffer()

        # listener. Important to spin a thread, otherwise the listen will block and no TF can be updated
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, spin_thread=True)

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

        encoded_img = self.cvbridge.imgmsg_to_cv2(img_msg=img, 
                                                  desired_encoding='passthrough')

        if modality == 'rgb':
            # save the image
            self._log_image(encoded_img, modality)

            # publish the image
            self.rgb_publisher.publish(img)
            
            '''
            only update json with RGB. We treat depth as an accessory
            '''
            self._json_update()

            # only increase the count with RGB so we don't double count
            self.rgb_count += 1

        elif modality == 'depth':
            self._log_image(encoded_img, modality)
            self.depth_publisher.publish(img)
            self.depth_count += 1

        return encoded_img
    
    def _log_image(self, img, modality):
        '''
        Save the image to disk
        '''
        # color the save log message in blue
        save_color_start = '\033[94m'
        save_color_reset = '\033[0m'

        if self.save_folder != '':
            if modality == 'rgb':
                cv2.imwrite(os.path.join(self.save_folder, modality, f'{modality}_{self.rgb_count}.png'), 
                            cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                np.save(os.path.join(self.save_folder, modality, f'{modality}_{self.rgb_count}.npy'), img)
            else:
                cv2.imwrite(os.path.join(self.save_folder, modality, f'{modality}_{self.depth_count}.png'), 
                            img)
                np.save(os.path.join(self.save_folder, modality, f'{modality}_{self.depth_count}.npy'), img)
            self.get_logger().info(f'{save_color_start}{modality} image saved{save_color_reset}')
        else:
            self.get_logger().info(f'{save_color_start}{modality} image captured, not saved{save_color_reset}')

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

    def _json_setup(self, calibration_path):
        '''
        Load the calibration yaml and set up the json we are about to dump.
        '''
        with open(calibration_path, "r") as f:
            config = yaml.safe_load(f)
            self.json_dict['w'] = config['w']
            self.json_dict['h'] = config['h']
            self.json_dict['fl_x'] = config['fl_x']
            self.json_dict['fl_y'] = config['fl_y']
            self.json_dict['cx'] = config['cx']
            self.json_dict['cy'] = config['cy']
            self.json_dict['k1'] = config['k1']
            self.json_dict['k2'] = config['k2']
            self.json_dict['p1'] = config['p1']
            self.json_dict['p2'] = config['p2']
            self.json_dict['camera_model'] = 'OPENCV'
            self.json_dict['frames'] = list()
            self.json_dict['applied_transform'] = np.array([[1.0, 0.0, 0.0, 0.0],
                                                            [0.0, 1.0, 0.0, 0.0],
                                                            [0.0, 0.0, 1.0, 0.0]], dtype=np.float64).tolist()

    def _json_update(self):

        # color the json log message in green
        json_color_start = '\033[92m'
        json_color_reset = '\033[0m'

        if self.json_path != '':
            self._json_update_helper()
            # overwrite the JSON file if there is one. Not sure if this is the best way to do it
            # potentially we can just keep the json_dict in memory and dump it at the end of the program
            # but this is a good way to keep the json file updated in case the program cannot exit as expected
            with open(self.json_path, 'wt') as f:
                json.dump(self.json_dict, f)
            self.get_logger().info(f'{json_color_start}JSON file updated{json_color_reset}')

    def _json_update_helper(self):
        '''
        Update the json dict with the latest transform
        '''
        update_dict = {}
        
        # get the coordinate of the camera in the base frame
        transformstamp = self.tf_buffer.lookup_transform(target_frame='link_base', 
                                            source_frame='link_realsense', 
                                            time=Time(), 
                                            timeout=Duration(seconds=2))
        transformation_matrix = self._process_tf(transformstamp)


        update_dict['file_path'] = os.path.join('camera/rgb', f'rgb_{self.rgb_count}.png')
        update_dict['transformation_matrix'] = transformation_matrix.tolist()
        update_dict['colmap_im_id'] = self.rgb_count
        update_dict['depth_file_path'] = os.path.join('camera/depth', f'depth_{self.rgb_count}.png')

        self.json_dict['frames'].append(update_dict)
        
    def _process_tf(self, transformstamp):
        '''
        Turn the transformstamp into a 4x4 transformation matrix

        Input: geometry_msgs.msg.TransformStamped
        Output: np.array(4x4)
        '''
        translation = np.array([transformstamp.transform.translation.x, transformstamp.transform.translation.y, transformstamp.transform.translation.z])
        quaternion = np.array([transformstamp.transform.rotation.x, transformstamp.transform.rotation.y, transformstamp.transform.rotation.z, transformstamp.transform.rotation.w])
        
        # convert quaternion to rotation matrix with scipy, which I think is more trustworthy than transforms3d
        rotation = Rotation.from_quat(quaternion)
        rotation_matrix = rotation.as_matrix()

        # create the 4x4 transformation matrix
        transformation_matrix = np.eye(4, dtype=np.float64)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation

        return transformation_matrix

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
            client.postprocess(rgb_response.frame, modality='rgb') # type: ignore
            client.postprocess(depth_response.frame, modality='depth') # type: ignore

            

        rclpy.spin_once(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
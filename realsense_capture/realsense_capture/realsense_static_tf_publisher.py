import sys
import numpy as np
import yaml

from transforms3d.quaternions import mat2quat

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class RealSenseStaticTFPublisher(Node):
    """
    Broadcast transforms that never change.
    """

    def __init__(self, transformation):
        super().__init__('realsense_static_tf_publisher') # type: ignore
        ############################ Launch Parameters ########################################

        # it's complicated why we need to load the config path instead of the content of config. See launch file for explanation
        self.declare_parameter(name = 'calibration_path', value = '')
        config_path = self.get_parameter('calibration_path').get_parameter_value().string_value
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
            self.mount_translation_y = config['mount_translation_y']
            self.mount_translation_z = config['mount_translation_z']
            self.d405_height = config['d405_height']
            self.d405_center2left = config['d405_center2left']

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish static transforms once at startup
        self.make_transforms()

    def make_transforms(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link_eef'
        t.child_frame_id = 'link_realsense'

        t.transform.translation.x = float(-self.d405_center2left)
        t.transform.translation.y = float(-self.mount_translation_y)
        t.transform.translation.z = float(self.mount_translation_z+self.d405_height)

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = RealSenseStaticTFPublisher(sys.argv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
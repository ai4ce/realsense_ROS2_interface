import yaml

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from rclpy.time import Time
from rclpy.duration import Duration

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RealSenseStaticTFPublisher(Node):
    """
    Broadcast transforms that never change.
    """

    def __init__(self):
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



        # ############################ TF Setup ########################################
        self.link_name = 'link_realsense'
        # Publish static transforms once at startup
        self.make_transforms()

        # buffer to hold the transform in a cache
        self.tf_buffer = Buffer()

        # listener. Important to spin a thread, otherwise the listen will block and no TF can be updated
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self, spin_thread=True)

        self.pose_publisher = self.create_publisher(
        msg_type=TransformStamped, 
        topic='/realsense_capture/realsense_pose', 
        qos_profile=10)

        self.create_timer(0.5, self.publish_pose)
    
    def publish_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('link_base', self.link_name, Time(), timeout=Duration(seconds=2))
            self.pose_publisher.publish(t)
        except Exception as e:
            self.get_logger().info(f"Failed to publish pose: {e}")

    def make_transforms(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link_eef'
        t.child_frame_id = self.link_name

        t.transform.translation.x = float(-self.d405_center2left)*0.001
        t.transform.translation.y = float(-self.mount_translation_y)*0.001
        t.transform.translation.z = float(self.mount_translation_z+self.d405_height)*0.001

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_static_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = RealSenseStaticTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
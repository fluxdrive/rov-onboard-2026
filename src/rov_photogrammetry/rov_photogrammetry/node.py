"""Photogrammetry + camera preview node. Polls /preview for low-res frames."""
import array
import urllib.request
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import CompressedImage

class PhotogrammetryNode(Node):
    def __init__(self):
        super().__init__('photogrammetry_node')
        self.declare_parameter('capture_url', 'http://192.168.7.2:8080/capture')
        self.declare_parameter('preview_url', 'http://192.168.7.2:8080/preview')
        self.declare_parameter('preview_rate', 2.0)
        self.capture_url = self.get_parameter('capture_url').value
        self.preview_url = self.get_parameter('preview_url').value
        rate = self.get_parameter('preview_rate').value
        self.image_pub = self.create_publisher(CompressedImage, '/photogrammetry/image', 10)
        self.preview_pub = self.create_publisher(CompressedImage, '/photogrammetry/preview', 1)
        self.capture_srv = self.create_service(Trigger, '/photogrammetry/capture', self.capture_cb)
        self.preview_timer = self.create_timer(1.0 / rate, self.fetch_preview)
        self.capture_count = 0
        self.preview_ok = False
        self.get_logger().info(f'Ready. Preview: {self.preview_url} @ {rate}Hz')

    def fetch_preview(self):
        try:
            with urllib.request.urlopen(self.preview_url, timeout=3) as resp:
                data = resp.read()
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_preview'
            msg.format = 'jpeg'
            msg.data = array.array('B', data)  # fast buffer copy; list(data) builds millions of ints
            self.preview_pub.publish(msg)
            if not self.preview_ok:
                self.preview_ok = True
                self.get_logger().info(f'Preview streaming ({len(data)} bytes/frame)')
        except Exception as e:
            if self.preview_ok:
                self.get_logger().warn(f'Preview error: {e}')
                self.preview_ok = False

    def capture_cb(self, request, response):
        self.get_logger().info('Capturing full-res...')
        try:
            with urllib.request.urlopen(f'{self.capture_url}?quality=95', timeout=15) as resp:
                data = resp.read()
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'photogrammetry_camera'
            msg.format = 'jpeg'
            msg.data = array.array('B', data)  # fast buffer copy; list(data) builds millions of ints
            self.image_pub.publish(msg)
            self.capture_count += 1
            response.success = True
            response.message = f'Capture #{self.capture_count}, {len(data)} bytes'
        except Exception as e:
            response.success = False
            response.message = f'Failed: {e}'
        return response

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PhotogrammetryNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

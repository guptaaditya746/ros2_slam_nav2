import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

class IMUSerialPublisher(Node):
    def __init__(self):
        super().__init__('imu_serial_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Adjust serial port as necessary
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            imu_msg = self.parse_imu_data(line)
            if imu_msg:
                self.publisher_.publish(imu_msg)
                self.get_logger().info(f'Published IMU data: {imu_msg}')

    def parse_imu_data(self, data):
        imu_msg = Imu()
        try:
            values = data.split(',')
            ax, ay, az = float(values[0]), float(values[1]), float(values[2])
            gx, gy, gz = float(values[3]), float(values[4]), float(values[5])

            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz
        except (IndexError, ValueError) as e:
            self.get_logger().warn(f'Error parsing IMU data: {data} -> {e}')
            return None
        return imu_msg

def main(args=None):
    rclpy.init(args=args)
    imu_serial_publisher = IMUSerialPublisher()
    rclpy.spin(imu_serial_publisher)
    imu_serial_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


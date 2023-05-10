import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
import numpy as np

import mindrove
from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from mindrove.data_filter import DataFilter, FilterTypes, AggOperations


class EmgPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publemg_publisher')

        BoardShim.enable_dev_board_logger()

        params = MindRoveInputParams()
        params.ip_port = 0
        params.serial_port = ''
        params.mac_address = ''
        params.serial_number = ''
        params.ip_address = ''
        params.ip_protocol = 0
        params.timeout = 0
        params.file = ''
        self.board = BoardShim(
            BoardIds.MINDROVE_WIFI_BOARD,
            params
        )
        self.board.prepare_session()
        # board.start_stream () # use this for default options
        self.board.start_stream(45000)

        self.publisher_ = self.create_publisher(Float64MultiArray, 'mindrove/emg', 10)

        self.emg_channels = BoardShim.get_emg_channels(BoardIds.MINDROVE_WIFI_BOARD)

        self.timer = self.create_timer(0.01, self.timer_callback)


    def timer_callback(self):
        msg = Float64MultiArray()
        data = self.board.get_current_board_data(1)
        emg_data = data[self.emg_channels].T[0]
        self.get_logger().info(f"data: {emg_data}")
        msg.data = emg_data.tolist()
        self.publisher_.publish(msg)


        
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = EmgPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
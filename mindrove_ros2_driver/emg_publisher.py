import rclpy
from rclpy.node import Node

from roboasset_msgs.msg import MindroveEmg
import numpy as np

import mindrove
from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from mindrove.data_filter import DataFilter, FilterTypes, AggOperations, NoiseTypes
from PyQt5 import QtGui, QtWidgets
import sys
import argparse
import logging
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.Qt import QtGui
from PyQt5 import QtGui

class EmgPublisher(Node):

    def __init__(self):
        super().__init__('emg_publisher')

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
        self.board.start_stream(450000)

        self.publisher_ = self.create_publisher(MindroveEmg, 'mindrove/emg', 10)

        self.emg_channels = BoardShim.get_emg_channels(BoardIds.MINDROVE_WIFI_BOARD)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.max = np.ones(len(self.emg_channels)) * -np.inf
        self.min = np.ones(len(self.emg_channels)) * np.inf
        
  


    def timer_callback(self):
        msg = MindroveEmg()
        
        data = self.board.get_current_board_data(1)
        board_id = BoardIds.MINDROVE_WIFI_BOARD.value
        for count, channel in enumerate(self.emg_channels):
           DataFilter.remove_environmental_noise(data[channel], BoardShim.get_sampling_rate(board_id), NoiseTypes.FIFTY.value)
        emg_data = data[self.emg_channels].T[0]
        msg.data = emg_data.tolist()
        self.max = np.maximum(self.max, emg_data)
        self.min = np.minimum(self.min, emg_data)
        msg.max = self.max.tolist()
        msg.min = self.min.tolist()

        self.publisher_.publish(msg)
        self.get_logger().info(f"data: {emg_data}")
        

        
def main(args=None):
    rclpy.init(args=args)

    emg_publisher = EmgPublisher()


    rclpy.spin(emg_publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    emg_publisher.destroy_node()
    rclpy.shutdown()



    params = MindRoveInputParams()

    try:
        board_shim = BoardShim(BoardIds.MINDROVE_WIFI_BOARD, params)
        board_shim.prepare_session()
        board_shim.start_stream()
        EmgPublisher(board_shim)
        rclpy.spin( EmgPublisher(board_shim))
    except BaseException:
        logging.warning('Exception', exc_info=True)
    finally:
        logging.info('End')
        if board_shim.is_prepared():
            logging.info('Releasing session')
            board_shim.stop_stream()
            board_shim.release_session()

if __name__ == '__main__':
    main()
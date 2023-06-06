import argparse
import logging
import sys

import mindrove
import numpy as np
import pyqtgraph as pg
import rclpy
from mindrove.board_shim import BoardIds, BoardShim, MindRoveInputParams
from mindrove.data_filter import (AggOperations, DataFilter, FilterTypes,
                                  NoiseTypes)
from PyQt5 import QtGui, QtWidgets
from pyqtgraph.Qt import QtCore, QtGui
from rclpy.node import Node

from roboasset_msgs.msg import MindroveEmg


class EmgPublisher(Node):

    def __init__(self):
        super().__init__('emg_publisher')

        BoardShim.enable_dev_board_logger()
        self.board_id = BoardIds.MINDROVE_WIFI_BOARD.value

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
        self.board.config_board(b'\x00\x00\x00\x00\x00')  # EMG

        self.publisher_ = self.create_publisher(MindroveEmg, 'mindrove/emg', 10)

        self.emg_channels = BoardShim.get_emg_channels(BoardIds.MINDROVE_WIFI_BOARD)
        self.sampling_rate = BoardShim.get_sampling_rate(self.board_id)
        self.sampling_period = 1 / self.sampling_rate * 2
        self.timer = self.create_timer(self.sampling_period, self.timer_callback)

        self.max = np.ones(len(self.emg_channels)) * -np.inf
        self.min = np.ones(len(self.emg_channels)) * np.inf


    def timer_callback(self):
        msg = MindroveEmg()
        
  
        data = self.board.get_current_board_data(1)
        for count, channel in enumerate(self.emg_channels):
            DataFilter.remove_environmental_noise(data[channel], self.sampling_rate, NoiseTypes.FIFTY.value)
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
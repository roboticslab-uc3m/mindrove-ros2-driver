import time

import mindrove
from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from mindrove.data_filter import DataFilter, FilterTypes, AggOperations


def main():
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
    board = BoardShim(
        BoardIds.MINDROVE_WIFI_BOARD,
        params
    )
    board.prepare_session()
    # board.start_stream () # use this for default options
    board.start_stream(45000)
    time.sleep(10)
    # data = board.get_current_board_data (256) # get latest 256 packages or less, doesnt remove them from internal buffer
    data = board.get_board_data()  # get all data and remove it from internal buffer
    board.stop_stream()
    board.release_session()

    print(data)


if __name__ == "__main__":
    main()
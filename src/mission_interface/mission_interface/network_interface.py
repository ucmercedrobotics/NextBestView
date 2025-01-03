import socket
import logging
import tempfile
import os
from typing import Tuple


class NetworkInterface:
    def __init__(
        self, logger: logging.Logger, log_directory: str, host="0.0.0.0", port=12345
    ):
        self.logger: logging.Logger = logger
        self.log_directory: str = log_directory
        # setting up server to listen
        self.host: str = host
        self.port: int = port
        self.max_rcv_bytes: int = 1024
        # Current mission file
        self.current_mission_file: str = None

        # creating logging directory
        os.makedirs(self.log_directory, exist_ok=True)

    def init_socket(self):
        self.server_socket: socket.socket = socket.socket(
            socket.AF_INET, socket.SOCK_STREAM
        )
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.logger.debug(f"Server listening on {self.host}:{self.port}")

    def close_socket(self):
        self.server_socket.close()
        self.logger.debug(f"Server closed on {self.host}:{self.port}")

    def receive_file(self) -> Tuple[int, str]:
        bytes_written: int = 0

        self.logger.debug(f"Waiting for client to connect to port {self.port}...")
        self.server_socket, addr = self.server_socket.accept()

        self.logger.debug(f"Connection from {addr}")

        with tempfile.NamedTemporaryFile(
            dir=self.log_directory, delete=False, mode="wb"
        ) as temp_file:
            self.current_mission_file = temp_file.name
            while True:
                chunk = self.server_socket.recv(self.max_rcv_bytes)
                if len(chunk) == 0:
                    break
                temp_file.write(chunk)
                bytes_written += len(chunk)

        return bytes_written, self.current_mission_file

    def send_acknowledgement(self):
        pass

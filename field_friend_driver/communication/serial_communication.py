#!/usr/bin/env python3
from functools import reduce
from operator import ixor
import os
from threading import Lock
from typing import List

import serial

from .communication import Communication


class SerialCommunication(Communication):
    """Handle serial communication."""

    def __init__(self, logger):
        super().__init__()
        self.logger = logger
        self.logger.info('Init serial communication')
        self.open_port()
        self.mutex = Lock()
        self._core_data = {
            'time': 0.0,
            'linear_speed': 0.0,
            'angular_speed': 0.0,
            'estop1_level': True,
            'estop2_level': True,
            'y_end_l': 0.0,
            'y_end_r': 0.0,
            'y_axis_idle': False,
            'y_axis_position': 0.0,
            'y_alarm_level': False,
            'z_end_t': 0.0,
            'z_end_b': 0.0,
            'z_axis_idle': False,
            'z_axis_position': 0.0,
            'z_alarm_level': False,
        }

    def enable(self):
        """
        Enable serial communication.

        Enable serial communication. There we need to call the flash
        python script from the lizard driver.
        """
        self.logger.info('Enable esp')
        command = '/root/.lizard/flash.py enable'
        os.system(command)
        self.logger.info('Esp is now enabled')

    def open_port(self):
        """Open port to device."""
        try:
            self.enable()
            # self.port.open()
            self.port = serial.Serial('/dev/esp', 115200)
        except serial.SerialException:
            self.logger.error('Could not open serial communication!')
            self.port = None

    def calculate_checksum(self, line: str) -> int:
        """Calculate checkusm of line."""
        return reduce(ixor, map(ord, line))

    def append_checksum(self, line: str) -> str:
        """Append checksum to the line."""
        checksum = self.calculate_checksum(line)
        line = f'{line}@{checksum:02x}\n'
        return line

    def send(self, line: str) -> None:
        """Send message to serial device."""
        # line = f"wheels.speed({cmd_msg.linear.x:3f}, {cmd_msg.angular.z:.3f})"
        if self.port is not None:
            line = self.append_checksum(line)
            self.mutex.acquire()
            self.port.write(line.encode())
            self.mutex.release()
        else:
            self.logger.warning('No Port open')

    def validate_checksum(self, line: str) -> bool:
        """Validate checksum."""
        line, checksum = line.split('@', 1)
        return self.calculate_checksum(line) == int(checksum, 16)

    def handle_core_message(self, words: List[str]) -> None:
        """Handle core message."""
        # self.logger.info(f'{words}')
        words.pop(0)
        self._core_data['time'] = int(words[0])
        self._core_data['linear_speed'] = float(words[1])
        self._core_data['angular_speed'] = float(words[2])
        self._core_data['estop1_level'] = bool(float(words[3]) > 0.5)
        self._core_data['estop2_level'] = bool((float(words[4]) > 0.5))
        self._core_data['y_end_l'] = float(words[5])
        self._core_data['y_end_r'] = float(words[6])
        self._core_data['y_axis_idle'] = bool(words[7])
        self._core_data['y_axis_position'] = float(words[8])
        self._core_data['y_alarm_level'] = bool(words[9])
        self._core_data['z_end_t'] = float(words[10])
        self._core_data['z_end_b'] = float(words[11])
        self._core_data['z_axis_idle'] = bool(words[12])
        self._core_data['z_axis_position'] = float(words[13])
        self._core_data['z_alarm_level'] = bool(words[14])
        self.notify_core_observers(self._core_data)

    def handle_expander_message(self, words: List[str]):
        """Handle expander message."""
        if len(words) < 2:
            return
        if words[1] == 'bms':
            self.notify_bms_observers(words[2:])

    def read(self) -> None:
        """Read from serial device."""
        try:
            self.mutex.acquire()
            buffer = self.port.read_all().decode(errors='replace')
        finally:
            self.mutex.release()

        # Split lines if we found multiple lines
        lines = buffer.split('\n')
        for line in lines:
            # self.logger.info(f'{line}')
            line = line.rstrip()
            if line[-3:-2] == '@' and line.count('@') == 1:
                if not self.validate_checksum(line):
                    return
                line = line[:-3]
            words = line.split()
            try:
                if not any(words):
                    return
                if words[0] == 'core':
                    self.handle_core_message(words)
                elif words[0] == 'expander:':
                    self.handle_expander_message(words)
                elif words[0] == 'error':
                    self.logger.error(f'{line}')
            except BaseException:
                self.logger.error(f'General exception in the following line: {
                                  line} from the following buffer {buffer}')

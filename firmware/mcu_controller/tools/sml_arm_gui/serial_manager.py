"""Threaded serial transport; all I/O stays off the GUI thread."""
from PySide6.QtCore import QThread, Signal, Slot
import serial
from serial.tools import list_ports


class SerialManager(QThread):
    received = Signal(str)
    transmitted = Signal(str)
    connection_changed = Signal(bool, str)
    failure = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        import queue
        self._queue = queue.Queue()
        self._port = None
        self._baud = 115200
        self._running = False

    @staticmethod
    def ports():
        return [(p.device, p.description) for p in list_ports.comports()]

    def connect_port(self, port, baud=115200):
        if self.isRunning():
            return False
        self._port = port
        self._baud = baud
        self._running = True
        self.start()
        return True

    def send(self, command):
        if self._running and self.isRunning():
            self._queue.put(command.strip())
            return True
        return False

    def disconnect_port(self):
        self._running = False

    def run(self):
        import queue
        try:
            with serial.Serial(self._port, self._baud, timeout=0.05, write_timeout=0.5) as device:
                self.connection_changed.emit(True, self._port)
                while self._running:
                    try:
                        while True:
                            command = self._queue.get_nowait()
                            device.write((command + '\n').encode('utf-8'))
                            self.transmitted.emit(command)
                    except queue.Empty:
                        pass
                    line = device.readline()
                    if line:
                        self.received.emit(line.decode('utf-8', errors='replace').strip())
        except (serial.SerialException, OSError) as exc:
            self.failure.emit(str(exc))
        finally:
            self._running = False
            while not self._queue.empty():
                try:
                    self._queue.get_nowait()
                except queue.Empty:
                    break
            self.connection_changed.emit(False, self._port or '')

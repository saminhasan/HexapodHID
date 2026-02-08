import time
import zlib
import struct
import serial
import numpy as np
from more_itertools import chunked
from threading import Thread, Event
from logging.handlers import RotatingFileHandler
import logging

from hexlink import Packet
from constants import *


def setup_logging(
    *,
    level: int = logging.INFO,
    logfile: str | None = "hexapod.log",
    max_bytes: int = 2_000_000,
    backups: int = 5,
) -> logging.Logger:
    fmt = "%(asctime)s %(levelname)s %(name)s: %(message)s"

    root = logging.getLogger()
    root.setLevel(level)
    root.handlers.clear()  # avoid duplicate handlers on reload

    sh = logging.StreamHandler()
    sh.setFormatter(logging.Formatter(fmt))
    root.addHandler(sh)

    if logfile:
        fh = RotatingFileHandler(logfile, maxBytes=max_bytes, backupCount=backups, encoding="utf-8")
        fh.setFormatter(logging.Formatter(fmt))
        root.addHandler(fh)

    return logging.getLogger("hexapod")


def traj_to_bytes_le(trajectory) -> bytes:
    b = bytearray()
    pack = struct.Struct("<6f").pack  # 6x float32, little-endian
    for row in trajectory:
        if len(row) != 6:
            raise ValueError("trajectory rows must have 6 values")
        b += pack(*map(float, row))
    return bytes(b)


class Hexapod:
    def __init__(self) -> None:
        self.log = logging.getLogger("hexapod")

        self.port = serial.Serial(port=None, timeout=None)
        self.byteBuffer = bytearray()

        self.inLoop = Event()
        self.listen = Event()

        self.start_us = time.perf_counter_ns() // 1000
        self.packet = Packet()

    def micros(self) -> int:
        return (time.perf_counter_ns() // 1000) - self.start_us

    @property
    def portStr(self) -> str | None:
        return self.port.port

    @property
    def is_connected(self) -> bool:
        return self.port.is_open

    def set_port(self, port: str) -> None:
        self.port.port = port

    def connect(self) -> None:
        if self.is_connected:
            self.log.info("[connect] Already connected to %s", self.portStr)
            return
        if not self.portStr:
            self.log.warning("[connect] No port specified")
            return

        try:
            self.port.open()
        except Exception:
            self.log.exception("[connect] Error connecting to %s", self.portStr)
            return

        self.log.info("[connect] Connected to %s", self.portStr)
        self.listen.set()
        Thread(target=self.run, daemon=True).start()

    def run(self) -> None:
        self.inLoop.set()
        self.log.debug("[run] listener started")

        try:
            while self.listen.is_set():
                # read at least 1 byte to block; if in_waiting has data, grab it all
                n = max(getattr(self.port, "in_waiting", 0), 1)
                data = self.port.read(n)
                if not data:
                    continue

                self.byteBuffer.extend(data)

                if b"\n" in self.byteBuffer:
                    *lines, rem = self.byteBuffer.split(b"\n")
                    for line in lines:
                        if line:
                            self.process_line(line)
                    self.byteBuffer = bytearray(rem)

        except Exception:
            self.log.exception("[run] listener crashed")
        finally:
            self.inLoop.clear()
            self.log.debug("[run] listener stopped")

    def process_line(self, line: bytes) -> None:
        # Typical device output is ASCII; keep it robust
        text = line.decode(errors="replace").strip()
        if text:
            self.log.info("[recv] %s", text)
        else:
            self.log.debug("[recv] (empty) %r", line)

    def sendData(self, data: bytes) -> bool:
        if not self.is_connected:
            self.log.warning("[send] Not connected")
            return False

        try:
            t0 = time.perf_counter_ns()
            sent = 0

            for c in chunked(data, 512):
                sent += self.port.write(bytes(c))
                # time.sleep(1e-3)

            dt = (time.perf_counter_ns() - t0) / 1e9

            ok = (sent == len(data))
            if sent > 64:
                self.log.info(
                    "[send] Sent %d bytes in %.2f s (%.2f MB/s)%s",
                    sent,
                    dt,
                    (sent / (dt * 1024 * 1024)) if dt > 0 else 0.0,
                    "" if ok else " [INCOMPLETE]",
                )
            elif not ok:
                self.log.warning("[send] Incomplete send (%d/%d)", sent, len(data))

            return ok

        except Exception:
            self.log.exception("[send] Error")
            return False

    def disconnect(self) -> None:
        self.listen.clear()

        if self.inLoop.is_set():
            try:
                self.port.cancel_read()
            except Exception:
                self.log.exception("[disconnect] cancel_read failed")

        if self.port.is_open:
            try:
                self.port.close()
                self.log.info("[disconnect] Disconnected from %s", self.portStr)
            except Exception:
                self.log.exception("[disconnect] Error closing %s", self.portStr)

    # ---- Commands ----

    def enable(self) -> None:
        self.sendData(self.packet.enable())

    def disable(self) -> None:
        self.sendData(self.packet.disable())

    def calibrate(self) -> None:
        self.sendData(self.packet.calibrate())

    def stage(self) -> None:
        self.sendData(self.packet.stage())

    def park(self) -> None:
        self.sendData(self.packet.park())

    def upload(self, filename: str) -> None:
        try:
            data = np.loadtxt(filename, delimiter=",")
            if data.ndim == 1:
                data = data.reshape(1, -1)

            if data.shape[1] != 6:
                self.log.error("[upload] File must have 6 columns, got %d", data.shape[1])
                return

            trajectory = data.tolist()
            self.sendData(self.packet.upload(trajectory))

            db = traj_to_bytes_le(trajectory)
            crc = zlib.crc32(db)
            
            # Debug: print first row bytes and CRC
            self.log.info("[upload] First row bytes: %s", db[:24].hex().upper())
            self.log.info("[upload] Total bytes: %d, CRC32: 0x%08X", len(db), crc)
            
            self.validate_trajectory(crc32=crc, length=len(trajectory))

            self.log.info("[upload] Uploaded %s (%d points)", filename, len(trajectory))

        except Exception:
            self.log.exception("[upload] Error loading %s", filename)

        except Exception:
            self.log.exception("[upload] Error loading %s", filename)

    def validate_trajectory(self, crc32: int = 0, length: int = 0) -> None:
        self.sendData(self.packet.validate_trajectory(crc32=crc32, length=length))

    def play(self) -> None:
        self.sendData(self.packet.play())

    def pause(self) -> None:
        self.sendData(self.packet.pause())

    def stop(self) -> None:
        self.sendData(self.packet.stop())

    def estop(self) -> None:
        self.sendData(self.packet.estop())

    def reset(self) -> None:
        self.sendData(self.packet.reset())

    def move(self, positions: list[float]) -> None:
        if isinstance(positions, np.ndarray):
            positions = positions.tolist()

        if not isinstance(positions, list):
            self.log.error("[move] positions must be list or ndarray")
            return

        self.sendData(self.packet.jog(positions))


if __name__ == "__main__":
    setup_logging(level=logging.INFO, logfile="hexapod.log")  # set DEBUG to get more noise

    hexapod = Hexapod()
    hexapod.set_port("COM11")
    hexapod.connect()

    time.sleep(10)

    hexapod.disconnect()









# import time
# import zlib
# import struct
# import serial
# import numpy as np
# from more_itertools import chunked
# from threading import Thread, Event
# from hexlink import Packet
# from constants import *
# import logging

# import logging
# from logging.handlers import RotatingFileHandler

# def setup_logging(logfile: str | None = "hexapod.log", level=logging.INFO) -> None:
#     fmt = "%(asctime)s %(levelname)s %(name)s: %(message)s"

#     root = logging.getLogger()
#     root.setLevel(level)
#     root.handlers.clear()  # avoid duplicate handlers in notebooks/reloads

#     sh = logging.StreamHandler()
#     sh.setFormatter(logging.Formatter(fmt))
#     root.addHandler(sh)

#     if logfile:
#         fh = RotatingFileHandler(logfile, maxBytes=2_000_000, backupCount=5, encoding="utf-8")
#         fh.setFormatter(logging.Formatter(fmt))
#         root.addHandler(fh)

# log = logging.getLogger(__name__)

# def traj_to_bytes_le(trajectory) -> bytes:
#     b = bytearray()
#     pack = struct.Struct('<6f').pack  # 6x float32, little-endian
#     for row in trajectory:
#         if len(row) != 6:
#             raise ValueError("trajectory rows must have 6 values")
#         b += pack(*map(float, row))
#     return bytes(b)
# class Hexapod:
#     def __init__(self) -> None:
#         self.port = serial.Serial(port=None, timeout=None)
#         self.byteBuffer = bytearray()
#         self.inLoop = Event()
#         self.listen = Event()
#         self.start_us = time.perf_counter_ns() // 1000
#         self.packet = Packet()  # Initialize packet builder

#     def micros(self):
#         return (time.perf_counter_ns() // 1000) - self.start_us

#     @property
#     def portStr(self) -> str | None:
#         return self.port.port

#     @property
#     def is_connected(self) -> bool:
#         return self.port.is_open

#     def set_port(self, port: str) -> None:
#         self.port.port = port

#     def connect(self) -> None:
#         if self.is_connected or not self.portStr:
#             print(f"[Hexapod.connect] : {'Already connected to ' + self.portStr if self.is_connected else 'No port specified.'}")
#             return
#         try:
#             self.port.open()
#         except Exception as e:
#             print(f"[Hexapod.connect] : Error connecting to {self.portStr} - {e}")
#             return

#         print(f"[Hexapod.connect] : Connected to {self.portStr}")
#         self.listen.set()
#         Thread(target=self.run, daemon=True).start()

#     def run(self) -> None:
#         self.inLoop.set()
#         while self.listen.is_set():
#             data = self.port.read(max(self.port.in_waiting, 1)) # change to min packet size
#             self.byteBuffer.extend(data)
#             if b'\n' in self.byteBuffer:
#                 *lines, rem = self.byteBuffer.split(b'\n')
#                 for line in lines:
#                     self.process_line(line)
#                 self.byteBuffer = bytearray(rem)
#         self.inLoop.clear()

#     def process_line(self, line: bytes) -> None:
#         print(f"[Hexapod.recvLine] : {line.decode(errors='ignore')}")
#         # add logging stuff here
#         pass

#     def sendData(self, data: bytes) -> bool:
#         if not self.is_connected:
#             print(f"[Hexapod.sendData] : Not connected")
#             return False
#         try:
#             t0 = time.perf_counter_ns()
#             sent = 0
#             for c in chunked(data, 1024):
#                 sent+=(self.port.write(bytes(c)) )
#                 time.sleep(1e-3)
#             t1 = time.perf_counter_ns()
#             dt = (t1 - t0) / 1e9
#             if sent > 64:
#                 print(f"[Hexapod.sendData] : Sent {sent} bytes in {dt:.2f} s ({sent  / (dt * 1024 * 1024):.2f} MB/s)")
#             # print(f"[Hexapod.sendData] :  {[f'0x{b:X}' for chunk in chunked(data, 1024) for b in chunk]}")
#             ok = sent == len(data)
#             if not ok:
#                 print(f"[Hexapod.sendData] : Incomplete send ({sent}/{len(data)})")
#             return ok
#         except Exception as e:
#             print(f"[Hexapod.sendData] : Error - {e}")
#             return False

#     def disconnect(self) -> None:
#         self.listen.clear()
#         if self.inLoop.is_set():
#             try: self.port.cancel_read()
#             except Exception as e: print(f"[Hexapod.disconnect] : cancel_read - {e}")
#         if self.port.is_open:
#             try:
#                 self.port.close()
#                 print(f"[Hexapod.disconnect] : Disconnected from {self.portStr}")
#             except Exception as e:
#                 print(f"[Hexapod.disconnect] : Error closing - {e}")


#     def enable(self) -> None:
#         # print("[Hexapod.enable] : Enable command sent.")
#         self.sendData(self.packet.enable())
    
#     def disable(self) -> None:
#         self.sendData(self.packet.disable())

#     def calibrate(self) -> None:
#         self.sendData(self.packet.calibrate())

#     def upload(self, filename: str) -> None:
#         # Load trajectory from file and convert to list[list[float]]
#         try:
#             # Assuming file contains numpy array or CSV-like data
#             data = np.loadtxt(filename, delimiter=',')
#             if data.ndim == 1:
#                 data = data.reshape(1, -1)
#             if data.shape[1] != 6:
#                 print(f"[Hexapod.upload] : Error - File must have 6 columns, got {data.shape[1]}")
#                 return
#             trajectory = data.tolist()
#             self.sendData(self.packet.upload(trajectory))
#             db = traj_to_bytes_le(trajectory) 
#             self.validate_trajectory(crc32=zlib.crc32(db), length=len(trajectory))
#             print(f"[Hexapod.upload] : Uploaded trajectory from {filename} with {len(trajectory)} points.")
#         except Exception as e:
#             print(f"[Hexapod.upload] : Error loading file - {e}")

#     def validate_trajectory(self, crc32: int = 0, length: int = 0) -> None:
#         self.sendData(self.packet.validate_trajectory(crc32=crc32, length=length))

#     def play(self) -> None:
#         self.sendData(self.packet.play())

#     def pause(self) -> None:
#         self.sendData(self.packet.pause())

#     def stop(self) -> None:
#         self.sendData(self.packet.stop())

#     def estop(self) -> None:
#         self.sendData(self.packet.estop())

#     def reset(self) -> None:
#         self.sendData(self.packet.reset())

#     def move(self, positions: list[float]) -> None:
#         # Old function called move(), new one is jog()
#         if not isinstance(positions, (list, np.ndarray)):
#             print(f"[Hexapod.move] : Error - positions must be list or ndarray")
#             return
#         if isinstance(positions, np.ndarray):
#             positions = positions.tolist()
#         self.sendData(self.packet.jog(positions))

# if __name__ == "__main__":
#     hexapod = Hexapod()
#     hexapod.set_port("COM11")
#     hexapod.connect()
#     import time
#     time.sleep(10)
#     # hexapod.sendData(b'Hello, Hexapod!\n')
#     hexapod.disconnect()
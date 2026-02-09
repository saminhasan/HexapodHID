"""
Decode and plot telemetry binary logs produced by TelemetryRecorder.

Usage:
    python decode_log.py                         # opens latest .bin in output/
    python decode_log.py output/telem_xxx.bin     # specific file

Each record is a raw 64-byte STATUS packet.
Payload layout (bytes 5..55, 51 bytes):
    [0..3]   uint32  timestamp (µs)
    [4]      uint8   globalFlags (bit0 = globalError)
    [5..27]  AxisTelemetry LEFT  (23 bytes)
    [28..50] AxisTelemetry RIGHT (23 bytes)

AxisTelemetry (23 bytes, packed little-endian):
    mode      uint8   (0=LOW, 1=HIGH)
    flags     uint8
    setpoint  float32
    theta     float32
    omega     float32
    tau       float32
    temp      int8
    rtt       uint16
    txErr     uint8
    timeouts  uint8
"""

import sys
import os
import glob
import struct
import numpy as np
import matplotlib.pyplot as plt

PACKET_SIZE = 64
MSGID_STATUS = 0xFF

# Struct formats (little-endian)
_HDR_FMT = "<IB"  # timestamp(u32), globalFlags(u8)  = 5 bytes
_AXIS_FMT = "<BBffffbHBB"  # 23 bytes per axis
_HDR_SIZE = struct.calcsize(_HDR_FMT)
_AXIS_SIZE = struct.calcsize(_AXIS_FMT)

# Slave names for the 6 axes
AXIS_LABELS = [
    "S1-L",
    "S1-R",
    "S2-L",
    "S2-R",
    "S3-L",
    "S3-R",
]


def decode_file(path: str) -> dict:
    """Read binary log and return dict of numpy arrays keyed by field name."""
    data = open(path, "rb").read()
    n_packets = len(data) // PACKET_SIZE
    if n_packets == 0:
        print(f"Empty log: {path}")
        sys.exit(1)

    print(f"Decoding {path}: {n_packets} packets, {len(data)} bytes")

    # Pre-allocate arrays
    timestamps = np.zeros(n_packets, dtype=np.float64)  # seconds
    slave_ids = np.zeros(n_packets, dtype=np.uint8)

    # Per-axis arrays: [n_packets] each
    fields = ["setpoint", "theta", "omega", "tau", "temperature", "rtt"]
    # Store per side: left and right
    left = {f: np.zeros(n_packets, dtype=np.float64) for f in fields}
    right = {f: np.zeros(n_packets, dtype=np.float64) for f in fields}

    for i in range(n_packets):
        pkt = data[i * PACKET_SIZE : (i + 1) * PACKET_SIZE]
        if pkt[4] != MSGID_STATUS:
            continue

        slave_ids[i] = pkt[1]
        payload = pkt[5:56]

        ts, _gflags = struct.unpack(_HDR_FMT, payload[:_HDR_SIZE])
        timestamps[i] = ts / 1e6  # µs → seconds

        # Left axis
        off = _HDR_SIZE
        _mode, _flags, sp, theta, omega, tau, temp, rtt, _txe, _to = struct.unpack(_AXIS_FMT, payload[off : off + _AXIS_SIZE])
        left["setpoint"][i] = sp
        left["theta"][i] = theta
        left["omega"][i] = omega
        left["tau"][i] = tau
        left["temperature"][i] = temp
        left["rtt"][i] = rtt

        # Right axis
        off += _AXIS_SIZE
        _mode, _flags, sp, theta, omega, tau, temp, rtt, _txe, _to = struct.unpack(_AXIS_FMT, payload[off : off + _AXIS_SIZE])
        right["setpoint"][i] = sp
        right["theta"][i] = theta
        right["omega"][i] = omega
        right["tau"][i] = tau
        right["temperature"][i] = temp
        right["rtt"][i] = rtt

    return {
        "timestamps": timestamps,
        "slave_ids": slave_ids,
        "left": left,
        "right": right,
        "n_packets": n_packets,
    }


def plot_telemetry(decoded: dict, title: str = "") -> None:
    """Plot all telemetry data, grouped by slave and axis."""
    ts = decoded["timestamps"]
    sids = decoded["slave_ids"]
    left = decoded["left"]
    right = decoded["right"]

    slaves = sorted(set(sids) - {0})  # exclude any zeros
    if not slaves:
        print("No valid slave packets found.")
        return

    # Relative time from first packet
    t0 = ts[ts > 0].min() if np.any(ts > 0) else 0
    t_rel = ts - t0

    plot_fields = [
        ("theta", "Position (rad)", "setpoint"),
        ("omega", "Velocity (rad/s)", None),
        ("tau", "Torque (Nm)", None),
        ("temperature", "Temperature (°C)", None),
    ]

    fig, axes = plt.subplots(len(plot_fields), 1, figsize=(14, 3 * len(plot_fields)), sharex=True)
    fig.suptitle(title or "Telemetry Log", fontsize=14, fontweight="bold")

    colors = {1: ("#1f77b4", "#aec7e8"), 2: ("#2ca02c", "#98df8a"), 3: ("#d62728", "#ff9896")}

    for ax, (field, ylabel, overlay) in zip(axes, plot_fields):
        for sid in slaves:
            mask = sids == sid
            t = t_rel[mask]
            c1, c2 = colors.get(sid, ("#333", "#999"))

            ax.plot(t, left[field][mask], color=c1, linewidth=0.5, label=f"S{sid}-L", alpha=0.8)
            ax.plot(t, right[field][mask], color=c2, linewidth=0.5, label=f"S{sid}-R", alpha=0.8)

            if overlay and overlay in left:
                ax.plot(t, left[overlay][mask], color=c1, linewidth=0.5, linestyle="--", alpha=0.4)
                ax.plot(t, right[overlay][mask], color=c2, linewidth=0.5, linestyle="--", alpha=0.4)

        ax.set_ylabel(ylabel)
        ax.legend(loc="upper right", fontsize=7, ncol=len(slaves) * 2)
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.show()


def find_latest_log(directory: str = "output") -> str | None:
    """Find the most recent .bin file in the output directory."""
    pattern = os.path.join(directory, "telem_*.bin")
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        filepath = find_latest_log()
        if not filepath:
            print("No .bin files found in output/. Pass a path as argument.")
            sys.exit(1)

    decoded = decode_file(filepath)
    plot_telemetry(decoded, title=os.path.basename(filepath))

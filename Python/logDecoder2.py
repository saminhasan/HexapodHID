import struct
import os
import pandas as pd
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

try:
    from tqdm import tqdm
except ImportError:
    def tqdm(iterable, **kwargs):
        return iterable

START_BYTE = 0xFE
END_BYTE   = 0xFF
PKT_LEN    = 64

def crc16_xmodem(data: bytes, init: int = 0x0000) -> int:
    # CRC-16/XMODEM: poly=0x1021, init=0x0000, refin=false, refout=false, xorout=0x0000
    crc = init
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc

def _parse_axis(buf: bytes, off: int) -> dict:
    mode  = buf[off + 0]
    flags = buf[off + 1]
    setpoint, theta, omega, tau = struct.unpack_from("<ffff", buf, off + 2)
    temp = struct.unpack_from("<b", buf, off + 18)[0]
    rtt  = struct.unpack_from("<H", buf, off + 19)[0]
    txErr = buf[off + 21]
    timeouts = buf[off + 22]
    return dict(mode=mode, flags=flags, setpoint=setpoint, theta=theta, omega=omega, tau=tau,
                temp=temp, rtt=rtt, txErr=txErr, timeouts=timeouts)

def extract_axis_dfs(bin_path: str, verify_crc: bool = True):
    """
    Returns: dict {axis_no: pandas.DataFrame}
      axis_no is computed as:
        axis A = 2*FROM_ID - 1
        axis B = 2*FROM_ID
    Assumptions: little-endian for uint32/uint16/float32 (common on embedded).
    """
    with open(bin_path, "rb") as f:
        data = f.read()

    rows_by_axis = {}  # axis_no -> list[dict]
    n = len(data)
    scan_limit = n - PKT_LEN + 1
    skip_until = 0

    for i in tqdm(range(max(scan_limit, 0)), desc="Parsing packets", unit="byte"):
        if i < skip_until:
            continue
        if data[i] != START_BYTE:
            continue
        if data[i + 63] != END_BYTE:
            continue

        pkt = data[i:i + PKT_LEN]

        # CRC16-XMODEM of bytes [0..60], stored at [61..62]
        if verify_crc:
            want = struct.unpack_from("<H", pkt, 61)[0]
            got = crc16_xmodem(pkt[0:61])
            if got != want:
                continue

        from_id = pkt[1]
        to_id   = pkt[2]
        seq     = pkt[3]
        msgid   = pkt[4]
        timestamp_us = struct.unpack_from("<I", pkt, 5)[0]
        resflag = pkt[9]

        axis1_no = 2 * from_id - 1
        axis2_no = 2 * from_id

        common = dict(from_id=from_id, to_id=to_id, seq=seq, msgid=msgid,
                      timestamp_us=timestamp_us, resflag=resflag)

        a1 = _parse_axis(pkt, 10)
        a2 = _parse_axis(pkt, 33)

        axis1_row = common.copy()
        axis1_row["axis_no"] = axis1_no
        axis1_row.update(a1)
        rows_by_axis.setdefault(axis1_no, []).append(axis1_row)

        axis2_row = common.copy()
        axis2_row["axis_no"] = axis2_no
        axis2_row.update(a2)
        rows_by_axis.setdefault(axis2_no, []).append(axis2_row)

        skip_until = i + PKT_LEN  # next packet (file is assumed packet-aligned once synced)

    # Build DataFrames
    dfs = {ax: pd.DataFrame(rows) for ax, rows in rows_by_axis.items()}

    # Optional: sort by time/seq if present
    for ax, df in dfs.items():
        if not df.empty:
            dfs[ax] = df.sort_values(["timestamp_us", "seq"], kind="stable").reset_index(drop=True)

    return dfs

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

root = tk.Tk()
root.withdraw()

bin_path = filedialog.askopenfilename(
    title="Select log file",
    filetypes=[("Binary files", "*.bin"), ("All files", "*.*")],
)

if not bin_path:
    print("No file selected. Exiting.")
    root.destroy()
    raise SystemExit(0)

dfs = extract_axis_dfs(bin_path, verify_crc=False)

def save_dfs_to_excel():
    default_name = f"{os.path.splitext(os.path.basename(bin_path))[0]}_axes.xlsx"
    out_path = filedialog.asksaveasfilename(
        title="Save Excel file",
        defaultextension=".xlsx",
        initialfile=default_name,
        filetypes=[("Excel Workbook", "*.xlsx")],
    )

    if not out_path:
        return

    try:
        with pd.ExcelWriter(out_path) as writer:
            for axis in range(1, 7):
                df = dfs.get(axis)
                if df is None:
                    pd.DataFrame().to_excel(writer, sheet_name=f"Axis_{axis}", index=False)
                else:
                    df.to_excel(writer, sheet_name=f"Axis_{axis}", index=False)

        messagebox.showinfo("Export complete", f"Saved Excel file:\n{out_path}")
    except Exception as exc:
        messagebox.showerror("Export failed", f"Could not save Excel file:\n{exc}")

root.deiconify()
root.title(f"Axis Plots - {os.path.basename(bin_path)}")
root.geometry("1280x720")

controls = ttk.Frame(root)
controls.pack(side="top", fill="x", padx=8, pady=8)

save_btn = ttk.Button(controls, text="Save as Excel", command=save_dfs_to_excel)
save_btn.pack(side="left")

notebook = ttk.Notebook(root)
notebook.pack(fill="both", expand=True)

for axis in range(1, 7):
    df = dfs.get(axis)

    tab = ttk.Frame(notebook)
    notebook.add(tab, text=f"Axis {axis}")

    if df is None or df.empty:
        msg = ttk.Label(tab, text=f"No data for axis {axis}")
        msg.pack(padx=12, pady=12)
        continue

    t = (df["timestamp_us"].to_numpy(dtype="float64")) * 1e-6  # seconds

    fig, ax = plt.subplots()
    ax.plot(t, df["setpoint"].to_numpy(), label="setpoint")
    ax.plot(t, df["theta"].to_numpy(), label="theta")
    ax.set_title(f"Axis {axis}: setpoint & theta vs time")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("rad")
    ax.grid(True)
    ax.legend()

    toolbar_frame = ttk.Frame(tab)
    toolbar_frame.pack(side="top", fill="x")

    canvas = FigureCanvasTkAgg(fig, master=tab)
    toolbar = NavigationToolbar2Tk(canvas, toolbar_frame)
    toolbar.update()
    canvas.draw()
    canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

root.mainloop()
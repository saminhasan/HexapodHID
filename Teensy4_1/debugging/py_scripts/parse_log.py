import re
from pathlib import Path
from typing import Any, Dict, List
import matplotlib.pyplot as plt

LOG_PATH = Path(__file__).with_name("log2.txt")

LINE_RE = re.compile(
    r"^cnt=(?P<cnt>-?\d+) "
    r"n=(?P<n>-?\d+) "
    r"idxBefore=(?P<idx_before>-?\d+) "
    r"idx=(?P<idx>-?\d+) "
    r"feedRate=(?P<feed_rate>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?) "
    r"frBefore=(?P<fr_before>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?) "
    r"frRemainder=(?P<fr_remainder>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?) "
    r"P0=\[(?P<p0>[^\]]+)\] "
    r"OUT\(fr=(?P<out_fr>[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\)=\[(?P<out>[^\]]+)\] "
    r"idx1=(?P<idx1>-?\d+) "
    r"P1=\[(?P<p1>[^\]]+)\]$"
)


def _parse_vector(raw: str) -> List[float]:
    return [float(tok) for tok in raw.strip().split()]


def parse_line(line: str) -> Dict[str, Any]:
    m = LINE_RE.match(line.strip())
    if not m:
        raise ValueError(f"Unrecognized log format: {line[:140]}")

    d = m.groupdict()
    return {
        "cnt": int(d["cnt"]),
        "n": int(d["n"]),
        "idx_before": int(d["idx_before"]),
        "idx": int(d["idx"]),
        "feed_rate": float(d["feed_rate"]),
        "fr_before": float(d["fr_before"]),
        "fr_remainder": float(d["fr_remainder"]),
        "p0": _parse_vector(d["p0"]),
        "out_fr": float(d["out_fr"]),
        "out": _parse_vector(d["out"]),
        "idx1": int(d["idx1"]),
        "p1": _parse_vector(d["p1"]),
    }


def build_columns(path: Path = LOG_PATH) -> Dict[str, List[Any]]:
    columns: Dict[str, List[Any]] = {
        "cnt": [],
        "n": [],
        "idx_before": [],
        "idx": [],
        "feed_rate": [],
        "fr_before": [],
        "fr_remainder": [],
        "out_fr": [],
        "idx1": [],
        # Vector columns as axis-wise columns for easy plotting/analysis
        "p0_0": [], "p0_1": [], "p0_2": [], "p0_3": [], "p0_4": [], "p0_5": [],
        "out_0": [], "out_1": [], "out_2": [], "out_3": [], "out_4": [], "out_5": [],
        "p1_0": [], "p1_1": [], "p1_2": [], "p1_3": [], "p1_4": [], "p1_5": [],
    }

    with path.open("r", encoding="utf-8") as f:
        for line in f:
            raw = line.strip()
            if not raw:
                continue
            row = parse_line(raw)
            columns["cnt"].append(row["cnt"])
            columns["n"].append(row["n"])
            columns["idx_before"].append(row["idx_before"])
            columns["idx"].append(row["idx"])
            columns["feed_rate"].append(row["feed_rate"])
            columns["fr_before"].append(row["fr_before"])
            columns["fr_remainder"].append(row["fr_remainder"])
            columns["out_fr"].append(row["out_fr"])
            columns["idx1"].append(row["idx1"])

            for i in range(6):
                columns[f"p0_{i}"].append(row["p0"][i])
                columns[f"out_{i}"].append(row["out"][i])
                columns[f"p1_{i}"].append(row["p1"][i])

    return columns


def main() -> None:
    pass


if __name__ == "__main__":
    data = build_columns()

    print(f"Loaded {len(data['cnt'])} rows from {LOG_PATH.name}")

    # Example column lookups.
    # print(f"data['idx'][:5] = {data['idx'][:5]}")
    # print(f"data['out_0'][:5] = {data['out_0'][:5]}")

    plt.figure()
    plt.plot(data["cnt"], data["feed_rate"], label="feed_rate")
    plt.xlabel("cnt")
    plt.ylabel("feed_rate")
    plt.legend()

    # Six extra figures: one per axis with idx, idx1, and out pose column.
    for axis in range(6):
        plt.figure()
        plt.plot(data["cnt"], data[f"p0_{axis}"], label="idx")
        plt.plot(data["cnt"], data[f"out_{axis}"], label=f"out_{axis}")
        plt.plot(data["cnt"], data[f"p1_{axis}"], label="idx1")

        plt.xlabel("cnt")
        plt.ylabel("value")
        plt.title(f"Axis {axis}: idx, idx1, out_{axis}")
        plt.legend()
    plt.figure()
    plt.plot(data["cnt"], data["fr_remainder"], label="fr_remainder")
    plt.plot(data["cnt"], data["feed_rate"], label="feed_rate")
    plt.xlabel("cnt")
    plt.ylabel("fr_remainder")
    plt.legend()
    plt.show()

    

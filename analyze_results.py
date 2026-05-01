#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "floorplanner-matplotlib"))

import matplotlib.pyplot as plt


METRICS = [
    ("objective", "Objective"),
    ("totalWirelength", "Wirelength"),
    ("chipArea", "Chip area"),
    ("runtimeSeconds", "Runtime seconds"),
]


def read_summary(path):
    with open(path) as f:
        data = json.load(f)
    result_dir = path.parent
    name = result_dir.name
    parent = result_dir.parent.name
    return {
        "resultDir": str(result_dir),
        "group": parent,
        "name": name,
        "mode": data.get("mode"),
        "solver": data.get("solver"),
        "feasible": bool(data.get("feasible")),
        "status": data.get("status"),
        "objective": data.get("objective"),
        "chipWidth": data.get("chipWidth"),
        "chipHeight": data.get("chipHeight"),
        "chipArea": data.get("chipArea"),
        "totalWirelength": data.get("totalWirelength"),
        "runtimeSeconds": data.get("runtimeSeconds"),
        "iterations": data.get("iterations"),
        "seed": data.get("seed"),
        "epochLength": data.get("epochLength"),
        "initialTemperature": data.get("initialTemperature"),
        "initialTemperatureUsed": data.get("initialTemperatureUsed"),
        "epochLengthUsed": data.get("epochLengthUsed"),
        "autoTemperature": data.get("autoTemperature"),
        "totalMoves": data.get("totalMoves"),
        "acceptedMoves": data.get("acceptedMoves"),
        "coolingRatio": data.get("coolingRatio"),
        "numBlocks": data.get("numBlocks"),
        "numNets": data.get("numNets"),
        "hasFixedOutline": data.get("hasFixedOutline"),
    }


def collect_results(root):
    rows = []
    for summary_path in sorted(Path(root).rglob("summary.json")):
        rows.append(read_summary(summary_path))
    return rows


def number(value):
    if value is None:
        return math.nan
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def write_csv(rows, path):
    fields = [
        "group",
        "name",
        "resultDir",
        "mode",
        "solver",
        "feasible",
        "status",
        "objective",
        "chipWidth",
        "chipHeight",
        "chipArea",
        "totalWirelength",
        "runtimeSeconds",
        "iterations",
        "seed",
        "epochLength",
        "initialTemperature",
        "initialTemperatureUsed",
        "epochLengthUsed",
        "autoTemperature",
        "totalMoves",
        "acceptedMoves",
        "coolingRatio",
        "numBlocks",
        "numNets",
        "hasFixedOutline",
    ]
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def write_markdown(rows, path):
    with open(path, "w") as f:
        f.write("# Floorplanner Result Summary\n\n")
        f.write(f"Total runs: {len(rows)}\n\n")
        feasible = sum(1 for row in rows if row["feasible"])
        f.write(f"Feasible runs: {feasible}\n\n")
        f.write("| Result | Mode | Solver | Feasible | Objective | Wirelength | Area | Runtime |\n")
        f.write("|---|---|---:|---:|---:|---:|---:|---:|\n")
        for row in rows:
            f.write(
                f"| {row['resultDir']} | {row.get('mode')} | {row.get('solver')} | "
                f"{row.get('feasible')} | {row.get('objective')} | "
                f"{row.get('totalWirelength')} | {row.get('chipArea')} | {row.get('runtimeSeconds')} |\n"
            )


def plot_metric(rows, metric, title, output_dir):
    feasible_rows = [row for row in rows if row["feasible"] and math.isfinite(number(row.get(metric)))]
    if not feasible_rows:
        return
    labels = [row["name"] for row in feasible_rows]
    values = [number(row.get(metric)) for row in feasible_rows]

    fig_w = max(8, min(24, 0.55 * len(labels) + 5))
    fig, ax = plt.subplots(figsize=(fig_w, 5.5))
    ax.bar(range(len(values)), values, color="#4c78a8")
    ax.set_title(title)
    ax.set_ylabel(metric)
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / f"{metric}.png", dpi=180)
    plt.close(fig)


def plot_feasibility(rows, output_dir):
    labels = [row["name"] for row in rows]
    values = [1 if row["feasible"] else 0 for row in rows]
    if not rows:
        return
    fig_w = max(8, min(24, 0.55 * len(labels) + 5))
    fig, ax = plt.subplots(figsize=(fig_w, 4.5))
    ax.bar(range(len(values)), values, color=["#54a24b" if v else "#e45756" for v in values])
    ax.set_title("Feasibility")
    ax.set_yticks([0, 1])
    ax.set_yticklabels(["infeasible", "feasible"])
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(output_dir / "feasibility.png", dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Analyze floorplanner result directories.")
    parser.add_argument("--root", default="out", help="Root directory to scan for summary.json files")
    parser.add_argument("--output-dir", default="results/comparisons", help="Directory for summary tables and plots")
    args = parser.parse_args()

    rows = collect_results(args.root)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    write_csv(rows, output_dir / "summary.csv")
    write_markdown(rows, output_dir / "summary.md")
    plot_feasibility(rows, output_dir)
    for metric, title in METRICS:
        plot_metric(rows, metric, title, output_dir)

    print(f"read {len(rows)} result summaries from {args.root}")
    print(f"wrote {output_dir / 'summary.csv'}")
    print(f"wrote {output_dir / 'summary.md'}")
    print(f"wrote comparison figures under {output_dir}")


if __name__ == "__main__":
    main()

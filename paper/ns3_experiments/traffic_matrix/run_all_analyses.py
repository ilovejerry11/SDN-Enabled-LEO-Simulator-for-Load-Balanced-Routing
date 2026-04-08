#!/usr/bin/env python3
"""
Run all analysis scripts in sequence:
1) tcp_throughput_analysis.py
2) isl_time_weighted_stats.py
3) gsl_time_weighted_stats.py
4) path_statistics_avg.py

The same input path is forwarded to each script.
Optionally passes an ID range only to the GSL analysis script.
"""

import argparse
import csv
import math
import re
import subprocess
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run all analysis scripts in sequence and summarize results."
    )
    parser.add_argument(
        "path",
        nargs="?",
        default=".",
        help="Input path passed to each script (default: current directory).",
    )
    parser.add_argument(
        "--id-range",
        nargs=2,
        type=int,
        metavar=("MIN_ID", "MAX_ID"),
        help="Inclusive endpoint ID range forwarded to gsl_time_weighted_stats.py only.",
    )
    parser.add_argument(
        "--stop-on-error",
        action="store_true",
        help="Stop immediately if any script fails.",
    )
    parser.add_argument(
        "--logs-root",
        default="analysis_logs",
        help="Directory where execution logs are written (default: analysis_logs).",
    )
    parser.add_argument(
        "--summary-csv",
        default=None,
        help=(
            "Path for metrics CSV summary output. "
            "Default: <logs_dir>/summary_results.csv"
        ),
    )

    args = parser.parse_args()

    if args.id_range and args.id_range[0] > args.id_range[1]:
        parser.error("Invalid --id-range: MIN_ID must be <= MAX_ID")

    return args


def build_steps(path_arg, id_range):
    gsl_args = [path_arg]
    if id_range:
        gsl_args.extend(["--id-range", str(id_range[0]), str(id_range[1])])

    return [
        {
            "name": "TCP Throughput Analysis",
            "script": "tcp_throughput_analysis.py",
            "args": [path_arg],
        },
        {
            "name": "Time-Weighted ISL Utilization",
            "script": "isl_time_weighted_stats.py",
            "args": [path_arg],
        },
        {
            "name": "Time-Weighted GSL Utilization",
            "script": "gsl_time_weighted_stats.py",
            "args": gsl_args,
        },
        {
            "name": "Path Statistics Averages",
            "script": "path_statistics_avg.py",
            "args": [path_arg],
        },
    ]


def run_step(step, base_dir, python_executable, log_dir):
    script_path = base_dir / step["script"]
    command = [python_executable, str(script_path), *step["args"]]
    log_file = log_dir / f"{script_path.stem}.log"

    result = {
        "name": step["name"],
        "script": str(script_path),
        "command": command,
        "log_file": str(log_file),
        "exit_code": 1,
        "duration_sec": 0.0,
        "success": False,
        "error": None,
    }

    if not script_path.exists():
        msg = f"Script not found: {script_path}"
        print(msg)
        with open(log_file, "w") as f:
            f.write(msg + "\n")
        result["error"] = msg
        return result

    print("\n" + "=" * 100)
    print(f"Running: {step['name']}")
    print(f"Command: {' '.join(command)}")
    print("=" * 100)

    started_at = time.time()

    try:
        with open(log_file, "w") as f:
            f.write(f"# Step: {step['name']}\n")
            f.write(f"# Command: {' '.join(command)}\n")
            f.write(f"# Started at: {datetime.now().isoformat()}\n\n")

            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )

            assert process.stdout is not None
            for line in process.stdout:
                print(line, end="")
                f.write(line)

            process.wait()
            result["exit_code"] = process.returncode

    except KeyboardInterrupt:
        result["error"] = "Interrupted by user"
        print("\nInterrupted by user.")
    except Exception as exc:
        result["error"] = str(exc)
        print(f"\nError while running step: {exc}")

    result["duration_sec"] = time.time() - started_at
    result["success"] = result["exit_code"] == 0 and result["error"] is None

    status = "SUCCESS" if result["success"] else "FAILED"
    print(f"\n[{status}] {step['name']} finished in {result['duration_sec']:.2f}s (exit={result['exit_code']})")
    print(f"Log file: {log_file}")

    return result


def print_final_summary(results, log_dir):
    print("\n" + "=" * 100)
    print("FINAL SUMMARY")
    print("=" * 100)
    print(f"{'Step':<35} {'Status':<10} {'Exit':<6} {'Time(s)':<10} {'Log'}")
    print(f"{'-' * 35} {'-' * 10} {'-' * 6} {'-' * 10} {'-' * 30}")

    for item in results:
        status = "OK" if item["success"] else "FAIL"
        log_name = Path(item["log_file"]).name
        print(
            f"{item['name']:<35} {status:<10} {item['exit_code']:<6} "
            f"{item['duration_sec']:<10.2f} {log_name}"
        )

    passed = sum(1 for item in results if item["success"])
    failed = len(results) - passed
    print("\nTotals:")
    print(f"  Passed: {passed}")
    print(f"  Failed: {failed}")
    print(f"  Logs:   {log_dir}")


def write_execution_summary_csv(results, csv_path):
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "step_name",
            "script",
            "status",
            "exit_code",
            "duration_sec",
            "log_file",
            "command",
            "error",
        ])

        for item in results:
            writer.writerow([
                item["name"],
                item["script"],
                "OK" if item["success"] else "FAIL",
                item["exit_code"],
                f"{item['duration_sec']:.2f}",
                item["log_file"],
                " ".join(item["command"]),
                item["error"] or "",
            ])


def discover_run_logs(path_arg):
    """Discover run folders and their logs_ns3 directories from a path argument."""
    raw_path = Path(path_arg)

    if path_arg == ".":
        runs_dir = (Path.cwd() / "runs").resolve()
        if not runs_dir.is_dir():
            return []
        return [
            (run_dir.name, run_dir / "logs_ns3")
            for run_dir in sorted(runs_dir.iterdir())
            if run_dir.is_dir() and run_dir.name.startswith("run_") and (run_dir / "logs_ns3").is_dir()
        ]

    if not raw_path.is_absolute():
        resolved_path = (Path.cwd() / raw_path).resolve()
    else:
        resolved_path = raw_path

    if not resolved_path.exists() or not resolved_path.is_dir():
        return []

    run_dirs = [
        run_dir
        for run_dir in sorted(resolved_path.iterdir())
        if run_dir.is_dir() and run_dir.name.startswith("run_") and (run_dir / "logs_ns3").is_dir()
    ]
    if run_dirs:
        return [(run_dir.name, run_dir / "logs_ns3") for run_dir in run_dirs]

    if resolved_path.name.startswith("run_") and (resolved_path / "logs_ns3").is_dir():
        return [(resolved_path.name, resolved_path / "logs_ns3")]

    if resolved_path.name == "logs_ns3":
        run_name = resolved_path.parent.name or "single_run"
        return [(run_name, resolved_path)]

    return []


def compute_flow_throughput_stats(logs_dir):
    """Compute mean/std/min throughput (Mbps) from tcp_flows data."""
    csv_file = logs_dir / "tcp_flows.csv"
    txt_file = logs_dir / "tcp_flows.txt"
    bandwidth_values = []

    if csv_file.exists():
        with open(csv_file, "r", newline="") as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 8:
                    continue
                try:
                    duration_ns = int(row[6])
                    sent_bytes = int(row[7])
                except (ValueError, TypeError):
                    continue
                if duration_ns <= 0:
                    continue

                sent_bits = sent_bytes * 8
                duration_seconds = duration_ns / 1e9
                throughput_mbps = (sent_bits / 1e6) / duration_seconds
                bandwidth_values.append(throughput_mbps)
    elif txt_file.exists():
        rate_pattern = re.compile(r"(\d+\.?\d*)\s*Mbit/s")
        with open(txt_file, "r") as f:
            for line in f:
                if "TCP Flow ID" in line or "---" in line:
                    continue
                match = rate_pattern.search(line)
                if match:
                    bandwidth_values.append(float(match.group(1)))

    if not bandwidth_values:
        return None

    n = len(bandwidth_values)
    mean_val = sum(bandwidth_values) / n
    std_val = math.sqrt(sum((x - mean_val) ** 2 for x in bandwidth_values) / n)
    min_val = min(bandwidth_values)

    return mean_val, std_val, min_val


def compute_path_metrics(logs_dir):
    """Compute average hop count and end-to-end distance from path_statistics.csv."""
    path_file = logs_dir / "path_statistics.csv"
    if not path_file.exists():
        return None

    total_hops = 0
    total_distance = 0.0
    valid_rows = 0

    with open(path_file, "r", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            return None

        if "hop_count" not in reader.fieldnames or "total_distance_m" not in reader.fieldnames:
            return None

        for row in reader:
            try:
                hop_count = int(row["hop_count"])
                distance_m = float(row["total_distance_m"])
            except (ValueError, TypeError, KeyError):
                continue

            total_hops += hop_count
            total_distance += distance_m
            valid_rows += 1

    if valid_rows == 0:
        return None

    return total_hops / valid_rows, total_distance / valid_rows


def compute_time_weighted_link_utilization(csv_file_path, id_min=None, id_max=None):
    """Compute mean/std of per-link time-weighted utilization."""
    if not csv_file_path.exists():
        return None

    link_weighted_sum = defaultdict(float)
    link_duration_sum = defaultdict(float)

    with open(csv_file_path, "r", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) != 5:
                continue

            try:
                net_dev_from = int(row[0])
                net_dev_to = int(row[1])
                start_time = int(row[2])
                end_time = int(row[3])
                utilization = float(row[4])
            except (ValueError, TypeError):
                continue

            if id_min is not None and (net_dev_from < id_min):
                continue
            if id_max is not None and (net_dev_from > id_max):
                continue

            duration = end_time - start_time
            link_id = (net_dev_from, net_dev_to)
            link_weighted_sum[link_id] += utilization * duration
            link_duration_sum[link_id] += duration

    per_link_values = []
    for link_id, weighted_sum in link_weighted_sum.items():
        duration_sum = link_duration_sum[link_id]
        if duration_sum > 0:
            per_link_values.append(weighted_sum / duration_sum)

    if not per_link_values:
        return None

    n = len(per_link_values)
    mean_val = sum(per_link_values) / n
    std_val = math.sqrt(sum((x - mean_val) ** 2 for x in per_link_values) / n)
    return mean_val, std_val


def to_csv_cell(value):
    if value is None:
        return ""
    return f"{value:.2f}"


def collect_metrics_rows(path_arg, id_range):
    run_logs = discover_run_logs(path_arg)
    rows = []

    id_min = id_range[0] if id_range else None
    id_max = id_range[1] if id_range else None

    for run_name, logs_dir in run_logs:
        bandwidth_stats = compute_flow_throughput_stats(logs_dir)
        path_metrics = compute_path_metrics(logs_dir)
        isl_stats = compute_time_weighted_link_utilization(logs_dir / "isl_utilization.csv")
        gsl_stats = compute_time_weighted_link_utilization(
            logs_dir / "gsl_utilization.csv",
            id_min=id_min,
            id_max=id_max,
        )

        row = {
            "run_name": run_name,
            "flow throughput avg. (Mbps)": to_csv_cell(bandwidth_stats[0]) if bandwidth_stats else "",
            "flow throughput std.": to_csv_cell(bandwidth_stats[1]) if bandwidth_stats else "",
            "flow throughput min.": to_csv_cell(bandwidth_stats[2]) if bandwidth_stats else "",
            "path avg. hop cnt.": to_csv_cell(path_metrics[0]) if path_metrics else "",
            "path avg. end-end dist.": to_csv_cell(path_metrics[1]) if path_metrics else "",
            "ISL link utilization avg. (%)": to_csv_cell(isl_stats[0] * 100.0) if isl_stats else "",
            "ISL link utilization std. (%)": to_csv_cell(isl_stats[1] * 100.0) if isl_stats else "",
            "GSL link utilization avg. (%)": to_csv_cell(gsl_stats[0] * 100.0) if gsl_stats else "",
            "GSL link utilization std. (%)": to_csv_cell(gsl_stats[1] * 100.0) if gsl_stats else "",
        }
        rows.append(row)

    return rows


def write_metrics_summary_csv(rows, csv_path):
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    headers = [
        "run_name",
        "flow throughput avg. (Mbps)",
        "flow throughput std.",
        "flow throughput min.",
        "path avg. hop cnt.",
        "path avg. end-end dist.",
        "ISL link utilization avg. (%)",
        "ISL link utilization std. (%)",
        "GSL link utilization avg. (%)",
        "GSL link utilization std. (%)",
    ]

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=headers)
        writer.writeheader()
        writer.writerows(rows)


def main():
    args = parse_args()

    base_dir = Path(__file__).resolve().parent
    logs_root = Path(args.logs_root)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_log_dir = logs_root / f"analysis_run_{timestamp}"
    run_log_dir.mkdir(parents=True, exist_ok=True)

    print(f"Base directory: {base_dir}")
    print(f"Input path: {args.path}")
    if args.id_range:
        print(f"GSL ID range: [{args.id_range[0]}, {args.id_range[1]}]")
    print(f"Logs directory: {run_log_dir}")

    steps = build_steps(args.path, args.id_range)
    results = []

    for step in steps:
        result = run_step(
            step=step,
            base_dir=base_dir,
            python_executable=sys.executable,
            log_dir=run_log_dir,
        )
        results.append(result)

        if args.stop_on_error and not result["success"]:
            print("Stopping early because --stop-on-error is enabled.")
            break

    if args.summary_csv:
        summary_csv_path = Path(args.summary_csv)
        if not summary_csv_path.is_absolute():
            summary_csv_path = base_dir / summary_csv_path
    else:
        summary_csv_path = run_log_dir / "summary_results.csv"

    metrics_rows = collect_metrics_rows(args.path, args.id_range)
    write_metrics_summary_csv(metrics_rows, summary_csv_path)

    execution_summary_path = run_log_dir / "execution_steps.csv"
    write_execution_summary_csv(results, execution_summary_path)

    print_final_summary(results, run_log_dir)
    print(f"Metrics CSV summary: {summary_csv_path}")
    print(f"Execution CSV summary: {execution_summary_path}")
    if not metrics_rows:
        print("Warning: No run_* folders found for metrics summary rows.")

    return 0 if all(item["success"] for item in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())

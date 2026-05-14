#!/usr/bin/env python3
"""
Analyze TCP flow RTT statistics from NS-3 simulation logs.
For each experiment:
1) Find all tcp_flow_<flow_id>_rtt.csv files in logs_ns3
2) Compute average RTT per flow
3) Compute average RTT across all flows in that experiment
4) Generate a CDF plot of the average RTTs
"""

import csv
import math
import os
import re
import sys
import numpy as np
import matplotlib.pyplot as plt

RTT_FILE_PATTERN = re.compile(r"^tcp_flow_(\d+)_rtt\.csv$")


def parse_rtt_file(rtt_file_path):
    """Parse one RTT CSV file and return average RTT (ns) and sample count."""
    rtt_values_ns = []

    try:
        with open(rtt_file_path, "r") as file:
            csv_reader = csv.reader(file)

            for row in csv_reader:
                if len(row) < 3:
                    continue

                try:
                    rtt_ns = float(row[2])
                except ValueError:
                    continue

                rtt_values_ns.append(rtt_ns)

    except Exception as e:
        print(f"Error reading RTT file {rtt_file_path}: {e}")
        return None

    if not rtt_values_ns:
        return None

    avg_rtt_ns = sum(rtt_values_ns) / len(rtt_values_ns)
    return {
        "avg_rtt_ns": avg_rtt_ns,
        "samples": len(rtt_values_ns),
    }


def process_single_experiment(logs_dir, run_name=None):
    """Process one logs_ns3 directory and compute RTT statistics."""

    if not os.path.exists(logs_dir):
        print(f"Error: logs directory '{logs_dir}' does not exist.")
        return None

    rtt_files = []
    for file_name in os.listdir(logs_dir):
        match = RTT_FILE_PATTERN.match(file_name)
        if match:
            flow_id = int(match.group(1))
            rtt_files.append((flow_id, os.path.join(logs_dir, file_name)))

    if not rtt_files:
        print(f"No RTT files found in {logs_dir}")
        return None

    rtt_files.sort(key=lambda x: x[0])

    flow_results = []

    for flow_id, rtt_file_path in rtt_files:
        stats = parse_rtt_file(rtt_file_path)
        if not stats:
            continue

        flow_results.append(
            {
                "flow_id": flow_id,
                "avg_rtt_ns": stats["avg_rtt_ns"],
                "avg_rtt_ms": stats["avg_rtt_ns"] / 1e6,
                "samples": stats["samples"],
            }
        )

    if not flow_results:
        print("No valid RTT samples found.")
        return None

    per_flow_avgs_ns = [item["avg_rtt_ns"] for item in flow_results]
    n = len(per_flow_avgs_ns)

    experiment_avg_rtt_ns = sum(per_flow_avgs_ns) / n
    variance = sum((x - experiment_avg_rtt_ns) ** 2 for x in per_flow_avgs_ns) / n
    std_rtt_ns = math.sqrt(variance)

    min_rtt_ns = min(per_flow_avgs_ns)
    max_rtt_ns = max(per_flow_avgs_ns)
    total_samples = sum(item["samples"] for item in flow_results)

    print("=" * 80)
    if run_name:
        print(f"TCP FLOW RTT ANALYSIS - {run_name}")
    else:
        print("TCP FLOW RTT ANALYSIS")
    print("=" * 80)

    print("Dataset Summary:")
    if run_name:
        print(f"  Experiment: {run_name}")
    print(f"  Logs dir: {logs_dir}")
    print(f"  RTT files found: {len(rtt_files):,}")
    print(f"  Flows with valid RTT: {n:,}")
    print(f"  Total RTT samples: {total_samples:,}")

    print("\nRTT Statistics (based on per-flow average RTT):")
    print(f"  Mean:               {experiment_avg_rtt_ns / 1e6:.6f} ms")
    print(f"  Standard Deviation: {std_rtt_ns / 1e6:.6f} ms")
    print(f"  Minimum:            {min_rtt_ns / 1e6:.6f} ms")
    print(f"  Maximum:            {max_rtt_ns / 1e6:.6f} ms")

    return {
        "run_name": run_name,
        "flow_count": n,
        "total_samples": total_samples,
        "avg_rtt_ns": experiment_avg_rtt_ns,
        "avg_rtt_ms": experiment_avg_rtt_ns / 1e6,
        "std_rtt_ms": std_rtt_ns / 1e6,
        "min_rtt_ms": min_rtt_ns / 1e6,
        "max_rtt_ms": max_rtt_ns / 1e6,
        "flow_avgs_ms": [x / 1e6 for x in per_flow_avgs_ns], # 保留所有 flow 的平均 RTT 供繪製 CDF 使用
    }


def process_all_experiments(runs_dir):
    """Process all experiment directories under runs_dir."""

    if not os.path.exists(runs_dir):
        print(f"Error: Runs directory '{runs_dir}' does not exist.")
        return []

    run_folders = [
        f
        for f in os.listdir(runs_dir)
        if os.path.isdir(os.path.join(runs_dir, f)) and f.startswith("run_")
    ]

    if not run_folders:
        print(f"No run folders found in {runs_dir}")
        return []

    run_folders.sort()

    print(f"Found {len(run_folders)} experiment folders in {runs_dir}")
    print("Processing each experiment...\n")

    results = []

    for run_folder in run_folders:
        logs_dir = os.path.join(runs_dir, run_folder, "logs_ns3")

        print(f"\n{'=' * 80}")
        print(f"Processing: {run_folder}")
        print(f"{'=' * 80}")

        if not os.path.exists(logs_dir):
            print(f"Warning: logs_ns3 directory not found in {run_folder}")
            continue

        try:
            result = process_single_experiment(logs_dir, run_folder)
            if result:
                results.append(result)
        except Exception as e:
            print(f"Error processing {run_folder}: {e}")
            continue

    if results:
        print_rtt_comparison(results)

    return results


def print_rtt_comparison(results):
    """Print comparative RTT analysis across all experiments."""

    print(f"\n\n{'=' * 120}")
    print("RTT COMPARISON ACROSS ALL EXPERIMENTS")
    print(f"{'=' * 120}")

    print(f"{'Experiment Name':<50} {'Flows':<8} {'Avg RTT(ms)':<12} {'Std(ms)':<10} {'Min(ms)':<10} {'Max(ms)':<10}")
    print(f"{'-' * 50} {'-' * 8} {'-' * 12} {'-' * 10} {'-' * 10} {'-' * 10}")

    for result in results:
        print(
            f"{result['run_name'] if result['run_name'] else 'Unknown':<50} "
            f"{result['flow_count']:>7,} "
            f"{result['avg_rtt_ms']:>11.3f} "
            f"{result['std_rtt_ms']:>9.3f} "
            f"{result['min_rtt_ms']:>9.3f} "
            f"{result['max_rtt_ms']:>9.3f}"
        )

    avg_rtts = [r["avg_rtt_ms"] for r in results]

    print(f"\n{'=' * 80}")
    print("CROSS-EXPERIMENT ANALYSIS")
    print(f"{'=' * 80}")

    print("Average RTT Across Experiments:")
    print(f"  Min: {min(avg_rtts):8.3f} ms")
    print(f"  Max: {max(avg_rtts):8.3f} ms")
    print(f"  Range: {max(avg_rtts) - min(avg_rtts):6.3f} ms")
    print(f"  Mean: {sum(avg_rtts)/len(avg_rtts):6.3f} ms")


def plot_rtt_cdf(results, output_filename="rtt_cdf.png"):
    """Generate and save a CDF plot of the average RTT per flow for all experiments."""
    if not results:
        return

    plt.figure(figsize=(10, 6))

    for result in results:
        data = result.get("flow_avgs_ms", [])
        if not data:
            continue

        # 計算 CDF
        x = np.sort(data)
        y = np.arange(1, len(x) + 1) / len(x)

        label_name = result["run_name"] if result["run_name"] else "Experiment"
        plt.plot(x, y, marker='.', linestyle='-', markersize=3, label=label_name)

    plt.title("CDF of Average RTT per Flow")
    plt.xlabel("Average RTT (ms)")
    plt.ylabel("CDF")
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.tight_layout()

    plt.savefig(output_filename, dpi=300)
    print(f"\n[+] CDF figure successfully saved to: {os.path.abspath(output_filename)}")


def main():
    if len(sys.argv) != 2:
        print("Usage:")
        print("  All experiments: python tcp_rtt_analysis.py <runs_directory>")
        print("  Current directory: python tcp_rtt_analysis.py .")
        print("  Single logs dir: python tcp_rtt_analysis.py <logs_ns3_directory>")
        return

    path = sys.argv[1]
    results = []

    if os.path.isdir(path):
        if path == ".":
            runs_path = os.path.join(path, "runs")
            if os.path.exists(runs_path):
                results = process_all_experiments(runs_path)
            else:
                print("No 'runs' directory found in current directory.")
        else:
            if any(
                f.startswith("run_")
                for f in os.listdir(path)
                if os.path.isdir(os.path.join(path, f))
            ):
                results = process_all_experiments(path)
            elif os.path.basename(path) == "logs_ns3":
                run_name = os.path.basename(os.path.dirname(path))
                single_result = process_single_experiment(path, run_name)
                if single_result:
                    results = [single_result]
            else:
                print(f"No run folders found in {path}")
    else:
        print(f"Error: '{path}' is not a valid directory.")

    # 如果有成功解析出結果，則繪製 CDF 圖
    if results:
        plot_rtt_cdf(results)


if __name__ == "__main__":
    main()
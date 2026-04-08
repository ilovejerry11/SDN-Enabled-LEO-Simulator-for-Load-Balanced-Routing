"""
This script runs the ns3 simulations for all the runs in the traffic matrix experiment.
"""

import exputil
import time
import os

local_shell = exputil.LocalShell()
max_num_processes = 3

# Check that no screen is running
if local_shell.count_screens() != 0:
    print("There is a screen already running. "
          "Please kill all screens before running this analysis script (killall screen).")
    exit(1)

# Read the folder name under runs/ 
runs_dir = "runs"
if not os.path.exists(runs_dir):
    print(f"Error: Runs directory '{runs_dir}' does not exist.")
    exit(1)

run_folders = [
    f for f in os.listdir(runs_dir)
    if os.path.isdir(os.path.join(runs_dir, f)) and f.startswith("run_")
]

if not run_folders:
    print(f"No run folders found in '{runs_dir}'.")
    exit(1)

print(f"Found {len(run_folders)} run folders in {runs_dir}")
print(run_folders)

# Generate the commands
commands_to_run = []
for run_folder in run_folders:
    logs_ns3_dir = os.path.join(runs_dir, run_folder, "logs_ns3")
    local_shell.remove_force_recursive(logs_ns3_dir)
    local_shell.make_full_dir(logs_ns3_dir)
    command = (
        f"cd ../../../ns3-sat-sim/simulator; "
        f"./waf --run=\"main_satnet "
        f"--run_dir='../../paper/ns3_experiments/traffic_matrix/runs/{run_folder}'\" "
        f"2>&1 | tee '../../paper/ns3_experiments/traffic_matrix/{logs_ns3_dir}/console.txt'"
    )
    commands_to_run.append(command)

# Run the commands
print(f"Running commands (at most {max_num_processes} in parallel)...")
for i, command in enumerate(commands_to_run):
    print(f"Starting command {i + 1} out of {len(commands_to_run)}: {command}")
    local_shell.detached_exec(command)
    while local_shell.count_screens() >= max_num_processes:
        time.sleep(2)

# Awaiting final completion before exiting
print(f"Waiting completion of the last {max_num_processes}...")
while local_shell.count_screens() > 0:
    time.sleep(2)
print("Finished.")
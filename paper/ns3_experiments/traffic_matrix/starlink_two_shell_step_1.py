import exputil
import networkload
import random
from collections import Counter

# Configuration parameters - easy to adjust
GS_NODE_START = 2502  # First ground station node ID
GS_NODE_END = 3502    # Last ground station node ID (exclusive)
FLOW_COUNTS = [100, 200, 300, 400, 500, 600]  # Different flow counts to test
FLOW_SIZE_BYTES = 1000000000000  # 1 TB
RANDOM_SEED = 123456789
START_TIME_INTERVAL_SECONDS = 0.1  # Interval between flow starts in seconds (reduced for more flows)

local_shell = exputil.LocalShell()

def generate_random_gs_flows(num_flows, available_nodes):
    """
    Generate random ground station flows.
    Allows any GS to be source/destination multiple times.
    Ensures src != dst for each flow.
    """
    print(f"  Generating {num_flows} random ground station flows...")
    
    random.seed(RANDOM_SEED)
    random.randint(0, 100000000)  # Legacy reasons
    seed_random = random.randint(0, 100000000)
    random.seed(seed_random)
    
    flows = []
    for i in range(num_flows):
        src_gs = random.choice(available_nodes)
        dst_gs = random.choice(available_nodes)
        
        # Ensure source and destination are different
        while src_gs == dst_gs:
            dst_gs = random.choice(available_nodes)
        
        flows.append((src_gs, dst_gs))
    
    # Calculate usage statistics
    sources = [flow[0] for flow in flows]
    destinations = [flow[1] for flow in flows]
    src_counter = Counter(sources)
    dst_counter = Counter(destinations)
    
    print(f"    Random approach statistics:")
    print(f"    - Source usage: min={min(src_counter.values()) if src_counter else 0}, max={max(src_counter.values()) if src_counter else 0}, avg={len(sources)/len(set(sources)) if sources else 0:.1f}")
    print(f"    - Dest usage: min={min(dst_counter.values()) if dst_counter else 0}, max={max(dst_counter.values()) if dst_counter else 0}, avg={len(destinations)/len(set(destinations)) if destinations else 0:.1f}")
    print(f"    - Unique GS used as source: {len(src_counter)}/{len(available_nodes)}")
    print(f"    - Unique GS used as destination: {len(dst_counter)}/{len(available_nodes)}")
    
    return flows

def generate_balanced_gs_flows(num_flows, available_nodes):
    """
    Generate balanced ground station flows.
    Each selected GS appears exactly once as source and exactly once as destination.
    Perfect 1:1 mapping with random permutation.
    """
    print(f"  Generating {num_flows} balanced ground station flows...")
    
    random.seed(RANDOM_SEED)
    random.randint(0, 100000000)  # Legacy reasons  
    seed_balanced = random.randint(0, 100000000)
    random.seed(seed_balanced)
    
    if len(available_nodes) < num_flows:
        print(f"    ERROR: Not enough ground stations ({len(available_nodes)}) for {num_flows} balanced flows")
        print(f"    Need at least {num_flows} ground stations for balanced approach")
        exit(1)
    
    # Select nodes for balanced flows
    selected_nodes = random.sample(available_nodes, num_flows)
    
    # Create lists of sources and destinations
    sources = selected_nodes.copy()
    destinations = selected_nodes.copy()
    
    # Shuffle destinations to create random pairing
    random.shuffle(destinations)
    
    # Create flows ensuring no self-loops (src != dst)
    flows = []
    max_attempts = num_flows * 2  # Prevent infinite loops
    attempts = 0
    
    for i, (src, dst) in enumerate(zip(sources, destinations)):
        if src == dst and attempts < max_attempts:
            # Find another destination to swap with
            for j in range(i + 1, len(destinations)):
                if destinations[j] != src and sources[j] != dst:
                    # Swap destinations
                    destinations[i], destinations[j] = destinations[j], destinations[i]
                    dst = destinations[i]
                    break
            attempts += 1
        
        flows.append((src, dst))
    
    print(f"    Balanced approach statistics:")
    print(f"    - Each selected GS used exactly once as source and destination")
    print(f"    - Perfect load balancing: 1 flow per selected GS")
    print(f"    - {num_flows} GS utilized out of {len(available_nodes)} available")
    
    return flows

def create_run_configuration(approach_name, num_flows, list_from_to):
    """
    Create run directory and configuration files for given approach and flow count.
    """
    print(f"  Creating run configuration for {approach_name} approach ({num_flows} flows)...")
    
    # Prepare run directory
    run_dir = f"runs/run_{num_flows}flows_{approach_name}_kuiper_isls_moving"
    local_shell.remove_force_recursive(run_dir)
    local_shell.make_full_dir(run_dir)
    
    # config_ns3.properties
    local_shell.copy_file("templates/template_config_ns3.properties", run_dir + "/config_ns3.properties")
    local_shell.sed_replace_in_file_plain(
        run_dir + "/config_ns3.properties",
        "[SATELLITE-NETWORK-FORCE-STATIC]",
        "false"  # Always moving satellites
    )
    
    # Update satellite network directory
    local_shell.sed_replace_in_file_plain(
        run_dir + "/config_ns3.properties",
        "[SATELLITE-NETWORK]",
        "starlink_2_shells_isls_plus_grid_ground_stations_298_algorithm_free_one_only_over_isls"
    )

    local_shell.sed_replace_in_file_plain(
        run_dir + "/config_ns3.properties",
        "[DYNAMIC-STATE]",
        "dynamic_state_1000ms_for_200s"
    )

    local_shell.sed_replace_in_file_plain(
        run_dir + "/config_ns3.properties",
        "[DYNAMIC-STATE-UPDATE-INTERVAL-NS]",
        "1000000000"  # 1000ms in nanoseconds
    )
    
    # Make logs_ns3 directory for console.txt mapping
    local_shell.make_full_dir(run_dir + "/logs_ns3")
    
    # .gitignore (legacy reasons)
    local_shell.write_file(run_dir + "/.gitignore", "logs_ns3")
    
    # Create flow sizes (all 1TB)
    flow_sizes = [FLOW_SIZE_BYTES] * len(list_from_to)
    
    # Create sequential start times with configurable interval
    start_times_seconds = [i * START_TIME_INTERVAL_SECONDS for i in range(len(list_from_to))]
    start_times = [t * 1000000000 for t in start_times_seconds]  # Convert to nanoseconds
    
    # Write the schedule
    networkload.write_schedule(
        run_dir + "/schedule_kuiper_630.csv",
        len(list_from_to),
        list_from_to,
        flow_sizes,
        start_times
    )
    
    # Update TCP flow logging configuration based on actual number of flows
    flow_ids = ",".join(str(i) for i in range(len(list_from_to)))
    local_shell.perfect_exec(
        f'sed -i "s/^tcp_flow_enable_logging_for_tcp_flow_ids=.*/tcp_flow_enable_logging_for_tcp_flow_ids=set({flow_ids})/" {run_dir}/config_ns3.properties'
    )
    
    # Create a flow summary file for reference
    import datetime
    flow_summary_content = f"# Flow Summary for {approach_name} approach ({num_flows} flows)\n"
    flow_summary_content += f"# Generated on: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
    flow_summary_content += f"# Configuration:\n"
    flow_summary_content += f"#   - Ground stations: {GS_NODE_START}-{GS_NODE_END-1} (node IDs)\n"
    flow_summary_content += f"#   - Number of flows: {num_flows}\n"
    flow_summary_content += f"#   - Flow size: {FLOW_SIZE_BYTES} bytes ({FLOW_SIZE_BYTES / 1e12:.1f} TB)\n"
    flow_summary_content += f"#   - Start interval: {START_TIME_INTERVAL_SECONDS}s between flows\n"
    flow_summary_content += f"#   - Random seed: {RANDOM_SEED}\n"
    flow_summary_content += f"#   - Approach: {approach_name}\n"
    flow_summary_content += f"#\n"
    flow_summary_content += f"# Format: Flow_ID,Source_GS,Dest_GS,Size_Bytes,Start_Time_ns\n"
    
    for i, (src, dst) in enumerate(list_from_to):
        start_time_ns = i * START_TIME_INTERVAL_SECONDS * 1000000000
        flow_summary_content += f"{i},{src},{dst},{FLOW_SIZE_BYTES},{int(start_time_ns)}\n"
    
    local_shell.write_file(run_dir + "/flow_summary.csv", flow_summary_content)
    
    print(f"    - Run directory: {run_dir}")
    print(f"    - Schedule: {run_dir}/schedule_kuiper_630.csv")
    print(f"    - Flow summary: {run_dir}/flow_summary.csv")
    print(f"    - TCP flow logging configured for flows 0-{len(list_from_to)-1}")
    
    return run_dir

def main():
    """Main execution function"""
    print("=" * 80)
    print("Multi-Flow Ground Station Traffic Configurations")
    print("=" * 80)
    print(f"Configuration:")
    print(f"  - Ground stations: {GS_NODE_START}-{GS_NODE_END-1} (node IDs)")
    print(f"  - Flow counts: {', '.join(map(str, FLOW_COUNTS))}")
    print(f"  - Flow size: {FLOW_SIZE_BYTES} bytes ({FLOW_SIZE_BYTES / 1e12:.1f} TB)")
    print(f"  - Start interval: {START_TIME_INTERVAL_SECONDS}s between flows")
    print(f"  - Random seed: {RANDOM_SEED}")
    print(f"  - Approaches: Random and Balanced")
    print(f"  - Network topology: 1000 ground stations (top_1000)")
    print()
    
    # Create list of all available ground station nodes
    available_nodes = list(range(GS_NODE_START, GS_NODE_END))
    print(f"Available ground stations: {len(available_nodes)} nodes ({min(available_nodes)}-{max(available_nodes)})")
    print()
    
    # Clean-up for a fresh run
    local_shell.remove_force_recursive("runs")
    local_shell.remove_force_recursive("pdf") 
    local_shell.remove_force_recursive("data")
    
    # Generate configurations for each flow count
    all_run_dirs = []
    
    for flow_count in FLOW_COUNTS:
        print(f"FLOW COUNT: {flow_count}")
        print("=" * 60)
        
        # Generate both approaches for this flow count
        # print(f"APPROACH 1: Random Ground Station Selection ({flow_count} flows)")
        # print("-" * 50)
        # flows_random = generate_random_gs_flows(flow_count, available_nodes)
        # run_dir_random = create_run_configuration("random", flow_count, flows_random)
        # all_run_dirs.append(("random", flow_count, run_dir_random))
        # print()
        
        print(f"APPROACH 2: Balanced Ground Station Selection ({flow_count} flows)")
        print("-" * 50)
        flows_balanced = generate_balanced_gs_flows(flow_count, available_nodes)
        run_dir_balanced = create_run_configuration("balanced", flow_count, flows_balanced)
        all_run_dirs.append(("balanced", flow_count, run_dir_balanced))
        print()
    
    # Summary
    print("=" * 80)
    print("SUMMARY")
    print("=" * 80)
    print(f"✓ Generated {len(all_run_dirs)} experiment configurations:")
    print()
    
    for approach, flow_count, run_dir in all_run_dirs:
        max_duration = (flow_count - 1) * START_TIME_INTERVAL_SECONDS
        print(f"  {flow_count:3d} flows ({approach:8s}): {run_dir}")
        print(f"      Duration: {max_duration:.1f}s (flows start 0.0s to {max_duration:.1f}s)")
    
    print()
    print("Configuration details:")
    print(f"  - Total ground stations available: {len(available_nodes)}")
    print(f"  - Flow sizes: {FLOW_SIZE_BYTES / 1e12:.1f} TB each")
    print(f"  - Start interval: {START_TIME_INTERVAL_SECONDS}s between flows")
    print(f"  - Network: Kuiper 630 satellites + 1000 ground stations")
    print()
    
    print("Flow distribution approaches:")
    print("  - Random: Any GS can be source/destination multiple times")
    print("  - Balanced: Each selected GS used exactly once as source and destination")
    print()
    
    print("Next steps:")
    print("  1. Run simulations using a step_2 script")
    print("  2. Compare performance across different flow counts")
    print("  3. Analyze random vs balanced approaches")
    print("  4. Generate scalability plots")
    print()
    
    print("Success - All multi-flow ground station traffic configurations generated!")
    print("=" * 80)

if __name__ == "__main__":
    main()
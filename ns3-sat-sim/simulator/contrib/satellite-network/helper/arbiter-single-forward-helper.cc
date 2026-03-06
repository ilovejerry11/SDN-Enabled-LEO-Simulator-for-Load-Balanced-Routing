/*
 * Copyright (c) 2020 ETH Zurich
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Simon               2020
 */

#include "arbiter-single-forward-helper.h"

namespace ns3 {

ArbiterSingleForwardHelper::ArbiterSingleForwardHelper (Ptr<BasicSimulation> basicSimulation, NodeContainer nodes) {
    std::cout << "SETUP SINGLE FORWARDING ROUTING" << std::endl;
    m_basicSimulation = basicSimulation;
    m_nodes = nodes;

    // Read in initial forwarding state
    std::cout << "  > Create initial single forwarding state" << std::endl;
    std::vector<std::map<std::pair<int32_t, int32_t>, std::tuple<int32_t, int32_t, int32_t>>> initial_forwarding_state = InitialEmptyForwardingState();
    basicSimulation->RegisterTimestamp("Create initial single forwarding state");

    // Set the routing arbiters
    std::cout << "  > Setting the routing arbiter on each node" << std::endl;
    for (size_t i = 0; i < m_nodes.GetN(); i++) {
        Ptr<ArbiterSingleForward> arbiter = CreateObject<ArbiterSingleForward>(m_nodes.Get(i), m_nodes, initial_forwarding_state[i]);
        m_arbiters.push_back(arbiter);
        m_nodes.Get(i)->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<Ipv4ArbiterRouting>()->SetArbiter(arbiter);
    }
    basicSimulation->RegisterTimestamp("Setup routing arbiter on each node");

    // Load first forwarding state
    m_dynamicStateUpdateIntervalNs = parse_positive_int64(m_basicSimulation->GetConfigParamOrFail("dynamic_state_update_interval_ns"));
    std::cout << "  > Forward state update interval: " << m_dynamicStateUpdateIntervalNs << "ns" << std::endl;
    std::cout << "  > Perform first forwarding state load for t=0" << std::endl;
    UpdateForwardingState(0);
    basicSimulation->RegisterTimestamp("Create initial single forwarding state");

    std::cout << std::endl;
}

ArbiterSingleForwardHelper::ArbiterSingleForwardHelper (Ptr<BasicSimulation> basicSimulation, NodeContainer nodes, Ptr<TopologySatelliteNetwork> topology) {
    std::cout << "SETUP SINGLE FORWARDING ROUTING WITH TOPOLOGY" << std::endl;
    m_basicSimulation = basicSimulation;
    m_nodes = nodes;
    m_topology = topology; // for updating the learning state

    // Read in initial forwarding state
    std::cout << "  > Create initial single forwarding state" << std::endl;
    std::vector<std::map<std::pair<int32_t, int32_t>, std::tuple<int32_t, int32_t, int32_t>>> initial_forwarding_state = InitialEmptyForwardingState();
    basicSimulation->RegisterTimestamp("Create initial single forwarding state");

    // Set the routing arbiters
    std::cout << "  > Setting the routing arbiter on each node" << std::endl;
    for (size_t i = 0; i < m_nodes.GetN(); i++) {
        Ptr<ArbiterSingleForward> arbiter = CreateObject<ArbiterSingleForward>(m_nodes.Get(i), m_nodes, initial_forwarding_state[i]);
        
        // Set up callback for on-demand route setup
        Callback<bool, int32_t, int32_t> routeSetupCallback = 
            MakeCallback(&ArbiterSingleForwardHelper::HandleRouteSetupRequest, this);
        arbiter->SetRouteSetupCallback(routeSetupCallback);
        
        m_arbiters.push_back(arbiter);
        m_nodes.Get(i)->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<Ipv4ArbiterRouting>()->SetArbiter(arbiter);
    }
    basicSimulation->RegisterTimestamp("Setup routing arbiter on each node");

    // Load first forwarding state
    m_dynamicStateUpdateIntervalNs = parse_positive_int64(m_basicSimulation->GetConfigParamOrFail("dynamic_state_update_interval_ns"));
    std::cout << "  > Forward state update interval: " << m_dynamicStateUpdateIntervalNs << "ns" << std::endl;
    std::cout << "  > Perform first forwarding state load for t=0" << std::endl;
    UpdateForwardingState(0);
    basicSimulation->RegisterTimestamp("Create initial single forwarding state");

    // Build neighbor cache for optimized routing
    BuildNeighborCache();

    std::cout << std::endl;
}

std::vector<std::map<std::pair<int32_t, int32_t>, std::tuple<int32_t, int32_t, int32_t>>>
ArbiterSingleForwardHelper::InitialEmptyForwardingState() {
    std::vector<std::map<std::pair<int32_t, int32_t>, std::tuple<int32_t, int32_t, int32_t>>> initial_forwarding_state;
    initial_forwarding_state.resize(m_nodes.GetN());
    return initial_forwarding_state;
}

void ArbiterSingleForwardHelper::UpdateForwardingState(int64_t t) {

    // Log current simulation time
    std::cout << "  > Update forwarding state at t=" << t << "ns" << std::endl;

    // Filename
    std::ostringstream res;
    res << m_basicSimulation->GetRunDir() << "/";
    res << m_basicSimulation->GetConfigParamOrFail("satellite_network_routes_dir") << "/fstate_" << t << ".txt";
    std::string filename = res.str();

    // Check that the file exists
    if (!file_exists(filename)) {
        throw std::runtime_error(format_string("File %s does not exist.", filename.c_str()));
    }

    // Open file
    std::string line;
    std::ifstream fstate_file(filename);
    if (fstate_file) {

        // Go over each line
        size_t line_counter = 0;
        while (getline(fstate_file, line)) {

            // Split on ,
            std::vector<std::string> comma_split = split_string(line, ",", 5);

            // Retrieve identifiers
            int64_t current_node_id = parse_positive_int64(comma_split[0]);
            int64_t target_node_id = parse_positive_int64(comma_split[1]);
            int64_t next_hop_node_id = parse_int64(comma_split[2]);
            int64_t my_if_id = parse_int64(comma_split[3]);
            int64_t next_if_id = parse_int64(comma_split[4]);

            // Check the node identifiers
            NS_ABORT_MSG_IF(current_node_id < 0 || current_node_id >= m_nodes.GetN(), "Invalid current node id.");
            NS_ABORT_MSG_IF(target_node_id < 0 || target_node_id >= m_nodes.GetN(), "Invalid target node id.");
            NS_ABORT_MSG_IF(next_hop_node_id < -1 || next_hop_node_id >= m_nodes.GetN(), "Invalid next hop node id.");

            // Drops are only valid if all three values are -1
            NS_ABORT_MSG_IF(
                    !(next_hop_node_id == -1 && my_if_id == -1 && next_if_id == -1)
                    &&
                    !(next_hop_node_id != -1 && my_if_id != -1 && next_if_id != -1),
                    "All three must be -1 for it to signify a drop."
            );

            // Check the interfaces exist
            NS_ABORT_MSG_UNLESS(my_if_id == -1 || (my_if_id >= 0 && my_if_id + 1 < m_nodes.Get(current_node_id)->GetObject<Ipv4>()->GetNInterfaces()), "Invalid current interface");
            NS_ABORT_MSG_UNLESS(next_if_id == -1 || (next_if_id >= 0 && next_if_id + 1 < m_nodes.Get(next_hop_node_id)->GetObject<Ipv4>()->GetNInterfaces()), "Invalid next hop interface");

            // Node id and interface id checks are only necessary for non-drops
            if (next_hop_node_id != -1 && my_if_id != -1 && next_if_id != -1) {

                // It must be either GSL or ISL
                bool source_is_gsl = m_nodes.Get(current_node_id)->GetObject<Ipv4>()->GetNetDevice(1 + my_if_id)->GetObject<GSLNetDevice>() != 0;
                bool source_is_isl = m_nodes.Get(current_node_id)->GetObject<Ipv4>()->GetNetDevice(1 + my_if_id)->GetObject<PointToPointLaserNetDevice>() != 0;
                NS_ABORT_MSG_IF((!source_is_gsl) && (!source_is_isl), "Only GSL and ISL network devices are supported");

                // If current is a GSL interface, the destination must also be a GSL interface
                NS_ABORT_MSG_IF(
                    source_is_gsl &&
                    m_nodes.Get(next_hop_node_id)->GetObject<Ipv4>()->GetNetDevice(1 + next_if_id)->GetObject<GSLNetDevice>() == 0,
                    "Destination interface must be attached to a GSL network device"
                );

                // If current is a p2p laser interface, the destination must match exactly its counter-part
                NS_ABORT_MSG_IF(
                    source_is_isl &&
                    m_nodes.Get(next_hop_node_id)->GetObject<Ipv4>()->GetNetDevice(1 + next_if_id)->GetObject<PointToPointLaserNetDevice>() == 0,
                    "Destination interface must be an ISL network device"
                );
                if (source_is_isl) {
                    Ptr<NetDevice> device0 = m_nodes.Get(current_node_id)->GetObject<Ipv4>()->GetNetDevice(1 + my_if_id)->GetObject<PointToPointLaserNetDevice>()->GetChannel()->GetDevice(0);
                    Ptr<NetDevice> device1 = m_nodes.Get(current_node_id)->GetObject<Ipv4>()->GetNetDevice(1 + my_if_id)->GetObject<PointToPointLaserNetDevice>()->GetChannel()->GetDevice(1);
                    Ptr<NetDevice> other_device = device0->GetNode()->GetId() == current_node_id ? device1 : device0;
                    NS_ABORT_MSG_IF(other_device->GetNode()->GetId() != next_hop_node_id, "Next hop node id across does not match");
                    NS_ABORT_MSG_IF(other_device->GetIfIndex() != 1 + next_if_id, "Next hop interface id across does not match");
                }

            }

            // Add to forwarding state
            // m_arbiters.at(current_node_id)->SetSingleForwardState(
            //     target_node_id,
            //     next_hop_node_id,
            //     1 + my_if_id,   // Skip the loop-back interface
            //     1 + next_if_id  // Skip the loop-back interface
            // );

            // Update reachability state for satellite-ground station connections
            if (m_topology != nullptr) {
                if (target_node_id == next_hop_node_id && next_hop_node_id != -1) {
                    // Direct connection - add to reachability
                    UpdateReachabilityState(current_node_id, target_node_id, true);
                } else {
                    // Indirect connection or drop - remove from reachability if it exists
                    UpdateReachabilityState(current_node_id, target_node_id, false);
                }
            }

            // Next line
            line_counter++;

        }

        // Close file
        fstate_file.close();

        // Note: Routes are now established on-demand via packet-in events,
        // so no need to pre-compute routes here

    } else {
        throw std::runtime_error(format_string("File %s could not be read.", filename.c_str()));
    }

    // Proactive route validation for active flows
    if (m_topology != nullptr) {
        ValidateActiveRoutes();
    }

    // Given that this code will only be used with satellite networks, this is okay-ish,
    // but it does create a very tight coupling between the two -- technically this class
    // can be used for other purposes as well
    if (!parse_boolean(m_basicSimulation->GetConfigParamOrDefault("satellite_network_force_static", "false"))) {

        // Plan the next update
        int64_t next_update_ns = t + m_dynamicStateUpdateIntervalNs;
        if (next_update_ns < m_basicSimulation->GetSimulationEndTimeNs()) {
            Simulator::Schedule(NanoSeconds(m_dynamicStateUpdateIntervalNs), &ArbiterSingleForwardHelper::UpdateForwardingState, this, next_update_ns);
        }

    }

}

void ArbiterSingleForwardHelper::UpdateReachabilityState(int64_t current_node_id, int64_t target_node_id, bool is_direct_connection) {
    // Only track satellite ↔ ground station connections
    bool current_is_satellite = m_topology->IsSatelliteId(current_node_id);
    bool target_is_ground_station = m_topology->IsGroundStationId(target_node_id);
    
    // Only process satellite → ground station connections
    if (!(current_is_satellite && target_is_ground_station)) {
        return;
    }

    if (is_direct_connection) {
        // Add bidirectional connection
        m_reachable_nodes[current_node_id].insert(target_node_id);
        m_reachable_nodes[target_node_id].insert(current_node_id);
        
        // // Debug output
        // std::cout << "  > Added reachability: Satellite " << current_node_id 
        //           << " ↔ Ground Station " << target_node_id << std::endl;
    } else {
        // Remove bidirectional connection if it exists
        auto it_current = m_reachable_nodes.find(current_node_id);
        if (it_current != m_reachable_nodes.end()) {
            auto erased_current = it_current->second.erase(target_node_id);
            if (erased_current > 0) {
                // Debug output only if we actually removed something
                // std::cout << "  > Removed reachability: Satellite " << current_node_id 
                //           << " ↔ Ground Station " << target_node_id << std::endl;
            }
            if (it_current->second.empty()) {
                m_reachable_nodes.erase(it_current);
            }
        }
        
        auto it_target = m_reachable_nodes.find(target_node_id);
        if (it_target != m_reachable_nodes.end()) {
            it_target->second.erase(current_node_id);
            if (it_target->second.empty()) {
                m_reachable_nodes.erase(it_target);
            }
        }
    }
}

std::pair<int64_t, int64_t> ArbiterSingleForwardHelper::FindInterfaceIds(int64_t from_node_id, int64_t to_node_id) {
    // Handle ground station to satellite or satellite to ground station connections
    bool from_is_satellite = m_topology->IsSatelliteId(from_node_id);
    bool to_is_satellite = m_topology->IsSatelliteId(to_node_id);
    bool from_is_ground_station = m_topology->IsGroundStationId(from_node_id);
    bool to_is_ground_station = m_topology->IsGroundStationId(to_node_id);
    
    // Case 1: Ground station to satellite
    if (from_is_ground_station && to_is_satellite) {
        return std::make_pair(0, 4); // GS uses interface 0, satellite uses interface 4
    }
    
    // Case 2: Satellite to ground station  
    if (from_is_satellite && to_is_ground_station) {
        return std::make_pair(4, 0); // Satellite uses interface 4, GS uses interface 0
    }
    
    // Case 3: Satellite to satellite (original ISL logic)
    if (from_is_satellite && to_is_satellite) {
        Ptr<Node> from_node = m_nodes.Get(from_node_id);
        Ptr<Ipv4> ipv4 = from_node->GetObject<Ipv4>();
        
        if (ipv4 == nullptr) {
            return std::make_pair(-1, -1);
        }
        
        // Iterate through all IPv4 interfaces (skip loopback at index 0)
        for (uint32_t i = 1; i < ipv4->GetNInterfaces(); i++) {
            Ptr<NetDevice> netdev = ipv4->GetNetDevice(i);
            
            // Check if it's a PointToPointLaserNetDevice (ISL)
            Ptr<PointToPointLaserNetDevice> p2pDevice = netdev->GetObject<PointToPointLaserNetDevice>();
            if (p2pDevice != nullptr) {
                // Get the destination node of this interface
                Ptr<Node> dest_node = p2pDevice->GetDestinationNode();
                if (dest_node != nullptr && dest_node->GetId() == to_node_id) {
                    // Found the interface! Now get the corresponding interface on the other end
                    Ptr<Channel> channel = p2pDevice->GetChannel();
                    if (channel != nullptr) {
                        Ptr<NetDevice> device0 = channel->GetDevice(0);
                        Ptr<NetDevice> device1 = channel->GetDevice(1);
                        
                        // Find the other device
                        Ptr<NetDevice> other_device = (device0->GetNode()->GetId() == from_node_id) ? device1 : device0;
                        
                        int64_t my_if_id = i - 1;  // Subtract 1 because we skip loopback (IPv4 interface index - 1)
                        int64_t next_if_id = other_device->GetIfIndex() - 1;  // NetDevice interface index - 1
                        
                        return std::make_pair(my_if_id, next_if_id);
                    }
                }
            }
        }
    }
    
    // No interface found or unsupported connection type
    return std::make_pair(-1, -1);
}

void ArbiterSingleForwardHelper::SetupGroundStationRoute(uint32_t gs_a, uint32_t gs_b, const std::vector<int32_t>& sat_path) {
    uint32_t num_satellites = m_topology->GetNumSatellites();
    uint32_t gs_a_node_id = num_satellites + gs_a;
    uint32_t gs_b_node_id = num_satellites + gs_b;

    if (sat_path.empty()) {
        m_arbiters.at(gs_a_node_id)->SetSingleForwardState(gs_a_node_id, gs_b_node_id, -1, -1, -1);
        return;
    }

    std::vector<int32_t> full_path;
    full_path.reserve(sat_path.size() + 2);
    full_path.push_back(gs_a_node_id);
    full_path.insert(full_path.end(), sat_path.begin(), sat_path.end());
    full_path.push_back(gs_b_node_id);

    for (size_t i = 1; i + 1 < full_path.size(); ++i) {
        int32_t node_id = full_path[i];
        if (m_topology->IsGroundStationId(node_id)) {
            int32_t gs_local_id = node_id - static_cast<int32_t>(num_satellites);
            if (gs_local_id < 0 || gs_local_id >= RELAY_GROUND_STATION_COUNT) {
                std::cout << "    >> WARNING: Non-relay GS" << gs_local_id
                          << " cannot be used as intermediate relay" << std::endl;
                m_arbiters.at(gs_a_node_id)->SetSingleForwardState(gs_a_node_id, gs_b_node_id, -1, -1, -1);
                return;
            }
        }
    }

    for (size_t i = 0; i + 1 < full_path.size(); ++i) {
        int64_t current_node = full_path[i];
        int64_t next_node = full_path[i + 1];

        auto interfaces = FindInterfaceIds(current_node, next_node);
        if (interfaces.first == -1) {
            std::cout << "    >> WARNING: Could not find interface Node" << current_node
                      << " -> Node" << next_node << std::endl;
            m_arbiters.at(current_node)->SetSingleForwardState(gs_a_node_id, gs_b_node_id, -1, -1, -1);
            return;
        }

        m_arbiters.at(current_node)->SetSingleForwardState(
            gs_a_node_id,
            gs_b_node_id,
            next_node,
            1 + interfaces.first,
            1 + interfaces.second
        );
    }
}

bool ArbiterSingleForwardHelper::HandleRouteSetupRequest(int32_t source_node_id, int32_t target_node_id) {
    // Check if this is a ground station to ground station flow
    bool src_is_gs = m_topology->IsGroundStationId(source_node_id);
    bool dst_is_gs = m_topology->IsGroundStationId(target_node_id);
    
    std::cout << "  > CONTROLLER: HandleRouteSetupRequest called: " << source_node_id 
              << " -> " << target_node_id << " (src_is_gs=" << src_is_gs 
              << ", dst_is_gs=" << dst_is_gs << ")" << std::endl;
    
    if (!src_is_gs || !dst_is_gs) {
        // Not a GS-to-GS flow, no route setup needed
        std::cout << "    >> WARNING: Not a GS-to-GS flow, no route setup needed" << std::endl;
        return false;
    }
    
    // Convert node IDs to ground station IDs
    uint32_t num_satellites = m_topology->GetNumSatellites();
    uint32_t gs_a = source_node_id - num_satellites;
    uint32_t gs_b = target_node_id - num_satellites;
    
    // Check if route exists and is still valid
    auto route_key = std::make_pair(gs_a, gs_b);
    if (m_cached_routes.find(route_key) != m_cached_routes.end()) {
        if (IsRouteStillValid(gs_a, gs_b)) {
            // Route still valid, no need to recalculate
            std::cout << "  > CONTROLLER: Using cached route GS" << gs_a << " -> GS" << gs_b << std::endl;
            return true;
        } else {
            // Route invalid, remove from cache
            m_cached_routes.erase(route_key);
            std::cout << "  > CONTROLLER: Cached route invalid, recalculating GS" << gs_a << " -> GS" << gs_b << std::endl;
        }
    }
    
    // Add to active flows
    m_active_gs_flows.insert(std::make_pair(gs_a, gs_b));
    
    SetupOptimalRoute(gs_a, gs_b);  // Immediate setup - now commented
    
    return true;
}

void ArbiterSingleForwardHelper::BuildNeighborCache() {
    if (m_neighbor_cache_built) return;
    
    std::cout << "  > Building ISL neighbor cache..." << std::endl;
    
    // Initialize cache for all nodes
    for (uint32_t i = 0; i < m_nodes.GetN(); i++) {
        m_neighbor_cache[i] = std::vector<std::tuple<int32_t, int32_t, int32_t>>();
    }
    
    // For each node, scan its interfaces to find neighbors
    for (uint32_t node_id = 0; node_id < m_nodes.GetN(); node_id++) {
        Ptr<Node> node = m_nodes.Get(node_id);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        
        if (ipv4 == nullptr) continue;
        
        for (uint32_t if_id = 1; if_id < ipv4->GetNInterfaces(); if_id++) {
            Ptr<NetDevice> netDevice = ipv4->GetNetDevice(if_id);
            Ptr<PointToPointLaserNetDevice> p2pDevice = 
                netDevice->GetObject<PointToPointLaserNetDevice>();
            
            if (p2pDevice != nullptr) {
                // Find the neighbor node connected to this interface
                Ptr<Channel> channel = p2pDevice->GetChannel();
                Ptr<PointToPointLaserChannel> p2pChannel = 
                    channel->GetObject<PointToPointLaserChannel>();
                
                if (p2pChannel != nullptr && p2pChannel->GetNDevices() == 2) {
                    // Get the other device on this channel
                    Ptr<NetDevice> otherNetDevice = 
                        (p2pChannel->GetDevice(0) == p2pDevice) ? 
                        p2pChannel->GetDevice(1) : p2pChannel->GetDevice(0);
                    
                    Ptr<PointToPointLaserNetDevice> otherDevice = 
                        otherNetDevice->GetObject<PointToPointLaserNetDevice>();
                    
                    if (otherDevice != nullptr) {
                        int32_t neighbor_id = otherDevice->GetNode()->GetId();
                        int32_t neighbor_if_id = FindInterfaceId(otherNetDevice);
                        
                        m_neighbor_cache[node_id].push_back(
                            std::make_tuple(neighbor_id, if_id, neighbor_if_id));
                    }
                }
            }
        }
    }
    
    m_neighbor_cache_built = true;
    std::cout << "  > ISL neighbor cache built successfully" << std::endl;
}

int32_t ArbiterSingleForwardHelper::FindInterfaceId(Ptr<NetDevice> target_device) {
    Ptr<Node> node = target_device->GetNode();
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    
    if (ipv4 == nullptr) return -1;
    
    for (uint32_t i = 1; i < ipv4->GetNInterfaces(); i++) {
        if (ipv4->GetNetDevice(i) == target_device) {
            return i;
        }
    }
    return -1;
}

const std::vector<std::tuple<int32_t, int32_t, int32_t>>& 
ArbiterSingleForwardHelper::GetNeighbors(int32_t node_id) {
    if (!m_neighbor_cache_built) {
        BuildNeighborCache();
    }
    
    static std::vector<std::tuple<int32_t, int32_t, int32_t>> empty_neighbors;
    auto it = m_neighbor_cache.find(node_id);
    if (it != m_neighbor_cache.end()) {
        return it->second;
    }
    return empty_neighbors;
}

double ArbiterSingleForwardHelper::GetLinkUtilization(int32_t from_node, int32_t to_node) {
    // Find the interface connecting from_node to to_node using neighbor cache
    const auto& neighbors = GetNeighbors(from_node);
    
    for (const auto& neighbor_tuple : neighbors) {
        int32_t neighbor_id = std::get<0>(neighbor_tuple);
        int32_t from_if_id = std::get<1>(neighbor_tuple);
        
        if (neighbor_id == to_node) {
            // Found the connection, get the network device
            Ptr<Node> node = m_nodes.Get(from_node);
            Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
            Ptr<NetDevice> netDevice = ipv4->GetNetDevice(from_if_id);
            Ptr<PointToPointLaserNetDevice> p2pDevice = 
                netDevice->GetObject<PointToPointLaserNetDevice>();
            
            if (p2pDevice != nullptr) {
                // Access the utilization vector using the getter (last available value)
                const std::vector<double>& util_history = p2pDevice->GetUtilizationHistory();
                return util_history.empty() ? 0.0 : util_history.back();
            }
        }
    }
    
    return 0.0; // No direct connection found
}

double ArbiterSingleForwardHelper::GetSatelliteGSLUtilization(int32_t from_node_id, int32_t to_node_id) {
    (void) to_node_id;

    bool from_is_satellite = m_topology->IsSatelliteId(from_node_id);
    bool from_is_ground_station = m_topology->IsGroundStationId(from_node_id);

    if (!from_is_satellite && !from_is_ground_station) {
        return 0.0;
    }

    Ptr<Node> node = m_nodes.Get(from_node_id);
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();

    uint32_t gsl_ipv4_if = from_is_satellite ? 5 : 1;
    if (ipv4 == nullptr || ipv4->GetNInterfaces() <= gsl_ipv4_if) {
        return 0.0;
    }

    Ptr<NetDevice> gsl_device = ipv4->GetNetDevice(gsl_ipv4_if);
    Ptr<GSLNetDevice> gsl_net_device = gsl_device->GetObject<GSLNetDevice>();

    if (gsl_net_device != nullptr) {
        const std::vector<double>& util_history = gsl_net_device->GetUtilizationHistory();
        return util_history.empty() ? 0.0 : util_history.back();
    }

    return 0.0;
}

double ArbiterSingleForwardHelper::CalculateGSLEdgeCost(int32_t from_node_id, int32_t to_node_id) {
    double utilization = GetSatelliteGSLUtilization(from_node_id, to_node_id);
    // if (utilization >= MAX_UTILIZATION) {
    //     return std::numeric_limits<double>::infinity();
    // }
    return COST_PARAM_A + COST_PARAM_GSL * utilization;
}

std::vector<std::pair<int32_t, double>> ArbiterSingleForwardHelper::GetNeighborsWithCost(int32_t node_id) {
    std::vector<std::pair<int32_t, double>> neighbors_with_cost;
    uint32_t num_satellites = m_topology->GetNumSatellites();

    auto isRelayGroundStationNode = [&](int32_t candidate_node_id) {
        if (!m_topology->IsGroundStationId(candidate_node_id)) {
            return false;
        }
        int32_t gs_local_id = candidate_node_id - static_cast<int32_t>(num_satellites);
        return gs_local_id >= 0 && gs_local_id < RELAY_GROUND_STATION_COUNT;
    };

    if (m_topology->IsSatelliteId(node_id)) {
        const auto& isl_neighbors = GetNeighbors(node_id);
        for (const auto& neighbor_tuple : isl_neighbors) {
            int32_t neighbor = std::get<0>(neighbor_tuple);
            if (!m_topology->IsSatelliteId(neighbor)) {
                continue;
            }
            double edge_cost = CalculateEdgeCost(node_id, neighbor);
            neighbors_with_cost.push_back(std::make_pair(neighbor, edge_cost));
        }
    }

    auto it = m_reachable_nodes.find(node_id);
    if (it != m_reachable_nodes.end()) {
        for (int64_t neighbor_id_64 : it->second) {
            int32_t neighbor_id = static_cast<int32_t>(neighbor_id_64);

            bool node_is_sat = m_topology->IsSatelliteId(node_id);
            bool node_is_gs = m_topology->IsGroundStationId(node_id);
            bool neighbor_is_sat = m_topology->IsSatelliteId(neighbor_id);
            bool neighbor_is_gs = m_topology->IsGroundStationId(neighbor_id);

            if ((node_is_sat && neighbor_is_gs) || (node_is_gs && neighbor_is_sat)) {
                if (node_is_sat && neighbor_is_gs && !isRelayGroundStationNode(neighbor_id)) {
                    continue;
                }
                if (node_is_gs && neighbor_is_sat && !isRelayGroundStationNode(node_id)) {
                    continue;
                }
                double edge_cost = CalculateGSLEdgeCost(node_id, neighbor_id);
                neighbors_with_cost.push_back(std::make_pair(neighbor_id, edge_cost));
            }
        }
    }

    return neighbors_with_cost;
}

double ArbiterSingleForwardHelper::CalculateEdgeCost(int32_t from_node, int32_t to_node) {
    double hop_cost = COST_PARAM_A; // Always 1 hop for direct neighbors
    
    // Lazy evaluation of utilization
    double utilization = GetLinkUtilization(from_node, to_node);
    // if (utilization >= MAX_UTILIZATION) {
    //     return std::numeric_limits<double>::infinity();
    // }
    
    // double util_cost = COST_PARAM_B * utilization / (1.0 - utilization); // Exponential cost function
    double util_cost = COST_PARAM_B * utilization; // Linear cost function
    return hop_cost + util_cost;
}

std::pair<std::vector<int32_t>, double> ArbiterSingleForwardHelper::ComputeMultiSourceDijkstra(
    const std::set<int64_t>& source_satellites,
    const std::set<int64_t>& target_satellites
) {
    std::map<int32_t, double> distances;
    std::map<int32_t, int32_t> previous;
    std::priority_queue<std::pair<double, int32_t>, 
                       std::vector<std::pair<double, int32_t>>, 
                       std::greater<std::pair<double, int32_t>>> pq;
    
    int32_t best_target_node = -1;
    double best_total_cost = std::numeric_limits<double>::infinity();
    
    // Initialize distances for all nodes (satellites + ground stations)
    for (uint32_t i = 0; i < m_nodes.GetN(); i++) {
        distances[i] = std::numeric_limits<double>::infinity();
        previous[i] = -1;
    }
    
    // Add all source satellites with distance 0
    for (int64_t src_sat : source_satellites) {
        distances[src_sat] = 0.0;
        pq.push(std::make_pair(0.0, src_sat));
    }
    
    // Dijkstra algorithm: evaluate target cost when target is popped
    while (!pq.empty()) {
        double curr_dist = pq.top().first;
        int32_t curr_node = pq.top().second;
        pq.pop();
        
        // Skip if we already found a better path to this node
        if (curr_dist > distances[curr_node]) continue;

        // If the frontier cannot beat the best completed target, stop
        if (best_target_node != -1 && curr_dist >= best_total_cost) {
            break;
        }

        // If this is a target satellite, include target-side GSL egress cost
        if (target_satellites.count(curr_node) > 0) {
            int32_t any_gs_node_id = static_cast<int32_t>(m_topology->GetNumSatellites());
            double candidate_total_cost = curr_dist + CalculateGSLEdgeCost(curr_node, any_gs_node_id);
            if (candidate_total_cost < best_total_cost) {
                best_total_cost = candidate_total_cost;
                best_target_node = curr_node;
            }
        }
        
        // Check all neighbors (ISL and GSL relay edges)
        const auto neighbors = GetNeighborsWithCost(curr_node);
        for (const auto& neighbor_cost_pair : neighbors) {
            int32_t neighbor = neighbor_cost_pair.first;
            double edge_cost = neighbor_cost_pair.second;
            
            // Skip infinite cost edges
            if (edge_cost == std::numeric_limits<double>::infinity()) continue;
            
            double new_dist = curr_dist + edge_cost;
            
            if (new_dist < distances[neighbor]) {
                distances[neighbor] = new_dist;
                previous[neighbor] = curr_node;
                pq.push(std::make_pair(new_dist, neighbor));
            }
        }
    }

    if (best_target_node != -1) {
        std::vector<int32_t> path;
        int32_t node = best_target_node;
        while (node != -1) {
            path.push_back(node);
            node = previous[node];
        }
        std::reverse(path.begin(), path.end());
        return std::make_pair(path, best_total_cost);
    }
    
    // No path found
    return std::make_pair(std::vector<int32_t>(), std::numeric_limits<double>::infinity());
}

bool ArbiterSingleForwardHelper::IsRouteStillValid(uint32_t gs_a, uint32_t gs_b) {
    auto route_it = m_cached_routes.find(std::make_pair(gs_a, gs_b));
    if (route_it == m_cached_routes.end()) return false;
    
    int64_t cached_first_sat = route_it->second.first;
    int64_t cached_last_sat = route_it->second.second;
    
    uint32_t gs_a_node_id = m_topology->GetNumSatellites() + gs_a;
    uint32_t gs_b_node_id = m_topology->GetNumSatellites() + gs_b;
    
    // Check if first and last satellites are still reachable
    auto it_a = m_reachable_nodes.find(gs_a_node_id);
    auto it_b = m_reachable_nodes.find(gs_b_node_id);
    
    if (it_a == m_reachable_nodes.end() || it_b == m_reachable_nodes.end()) {
        return false;
    }
    
    bool first_sat_reachable = it_a->second.count(cached_first_sat) > 0;
    bool last_sat_reachable = it_b->second.count(cached_last_sat) > 0;
    
    return first_sat_reachable && last_sat_reachable;
}

void ArbiterSingleForwardHelper::ValidateActiveRoutes() {
    std::cout << "  > Validating " << m_active_gs_flows.size() << " active ground station flows..." << std::endl;
    
    std::vector<std::pair<uint32_t, uint32_t>> flows_to_recalculate;
    
    // Check all active flows for route validity
    for (const auto& flow : m_active_gs_flows) {
        uint32_t gs_a = flow.first;
        uint32_t gs_b = flow.second;
        
        if (!IsRouteStillValid(gs_a, gs_b)) {
            flows_to_recalculate.push_back(flow);
            std::cout << "    >> Route GS" << gs_a << " -> GS" << gs_b << " is no longer valid" << std::endl;
        }
    }
    
    // Recalculate invalid routes
    for (const auto& flow : flows_to_recalculate) {
        uint32_t gs_a = flow.first;
        uint32_t gs_b = flow.second;
        
        // Clear existing forwarding state for this flow
        uint32_t gs_a_node_id = m_topology->GetNumSatellites() + gs_a;
        uint32_t gs_b_node_id = m_topology->GetNumSatellites() + gs_b;
        m_arbiters.at(gs_a_node_id)->SetSingleForwardState(gs_a_node_id, gs_b_node_id, -1, -1, -1);
        
        // Remove from cache
        auto route_key = std::make_pair(gs_a, gs_b);
        m_cached_routes.erase(route_key);

        SetupOptimalRoute(gs_a, gs_b);
    }
    
    if (flows_to_recalculate.empty()) {
        std::cout << "    >> All active routes remain valid" << std::endl;
    } else {
        std::cout << "    >> Recalculated " << flows_to_recalculate.size() << " invalid routes" << std::endl;
    }
}

void ArbiterSingleForwardHelper::SetupOptimalRoute(uint32_t gs_a, uint32_t gs_b) {
    uint32_t num_satellites = m_topology->GetNumSatellites();
    uint32_t gs_a_node_id = num_satellites + gs_a;
    uint32_t gs_b_node_id = num_satellites + gs_b;
    
    // Find reachable satellites for both ground stations
    auto it_a = m_reachable_nodes.find(gs_a_node_id);
    auto it_b = m_reachable_nodes.find(gs_b_node_id);
    
    if (it_a == m_reachable_nodes.end() || it_b == m_reachable_nodes.end() ||
        it_a->second.empty() || it_b->second.empty()) {
        // No reachable satellites for one or both ground stations
        std::cout << "    >> WARNING: No reachable satellites for GS" << gs_a << " -> GS" << gs_b 
                  << ", setting drop entry" << std::endl;
        m_arbiters.at(gs_a_node_id)->SetSingleForwardState(gs_a_node_id, gs_b_node_id, -1, -1, -1);
        return;
    }
    
    // Use multi-source Dijkstra to find optimal path
    std::pair<std::vector<int32_t>, double> result = 
        ComputeMultiSourceDijkstra(it_a->second, it_b->second);
    
    std::vector<int32_t> path = result.first;
    double cost = result.second;
    
    if (path.empty() || cost == std::numeric_limits<double>::infinity()) {
        // No path exists between any satellite pairs
        std::cout << "    >> WARNING: No satellite path for GS" << gs_a << " -> GS" << gs_b 
                  << ", setting drop entry" << std::endl;
        m_arbiters.at(gs_a_node_id)->SetSingleForwardState(gs_a_node_id, gs_b_node_id, -1, -1, -1);
        return;
    }
    
    // Extract first and last satellites from the computed path
    int64_t first_sat = path.front();
    int64_t last_sat = path.back();
    
    // Cache the route
    m_cached_routes[std::make_pair(gs_a, gs_b)] = std::make_pair(first_sat, last_sat);

    SetupGroundStationRoute(gs_a, gs_b, path);
    
    std::cout << "    >> Optimal route GS" << gs_a << " -> GS" << gs_b
              << " via entry Sat" << first_sat << " -> exit Sat" << last_sat
              << " (cost: " << cost << ", hops: " << path.size() + 1 << ")" << std::endl;
    
    // Print the complete path (including possible GS relays)
    std::cout << "    >> Complete path: GS" << gs_a;
    for (const auto& node_id : path) {
        if (m_topology->IsSatelliteId(node_id)) {
            std::cout << " -> Sat" << node_id;
        } else {
            std::cout << " -> GS" << (node_id - static_cast<int32_t>(num_satellites));
        }
    }
    std::cout << " -> GS" << gs_b << std::endl;
}

} // namespace ns3

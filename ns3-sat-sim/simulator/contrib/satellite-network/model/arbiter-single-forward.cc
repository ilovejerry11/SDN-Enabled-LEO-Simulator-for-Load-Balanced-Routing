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

#include "arbiter-single-forward.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (ArbiterSingleForward);
TypeId ArbiterSingleForward::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::ArbiterSingleForward")
            .SetParent<ArbiterSatnet> ()
            .SetGroupName("BasicSim")
    ;
    return tid;
}

ArbiterSingleForward::ArbiterSingleForward(
        Ptr<Node> this_node,
        NodeContainer nodes,
        std::map<std::pair<int32_t, int32_t>, std::tuple<int32_t, int32_t, int32_t>> next_hop_list
) : ArbiterSatnet(this_node, nodes)
{
    m_next_hop_list = next_hop_list;
}

std::tuple<int32_t, int32_t, int32_t> ArbiterSingleForward::TopologySatelliteNetworkDecide(
        int32_t source_node_id,
        int32_t target_node_id,
        Ptr<const Packet> pkt,
        Ipv4Header const &ipHeader,
        bool is_request_for_source_ip_so_no_next_header
) {
    auto key = std::make_pair(source_node_id, target_node_id);
    auto it = m_next_hop_list.find(key);
    std::tuple<int32_t, int32_t, int32_t> result = (it != m_next_hop_list.end())
            ? it->second
            : std::make_tuple(-2, -2, -2);
    
    // If no route exists yet (-2 means no route set up), trigger immediate optimal route setup
    if (std::get<0>(result) == -2 && !m_route_setup_callback.IsNull()) {
        // Check if route setup already in progress for this destination
        if (m_pending_route_setups.find(target_node_id) == m_pending_route_setups.end()) {
            // Mark as pending to avoid duplicate route setups
            m_pending_route_setups.insert(target_node_id);
            
            // Set up optimal route immediately on first packet arrival
            if (m_route_setup_callback(source_node_id, target_node_id)) {
                // Route setup successful, get the new route
                it = m_next_hop_list.find(key);
                result = (it != m_next_hop_list.end())
                        ? it->second
                        : std::make_tuple(-2, -2, -2);
                std::cout << "    >> CONTROL: First packet to " << target_node_id 
                          << ", optimal route set up immediately: (" << std::get<0>(result) 
                          << "," << std::get<1>(result) << "," << std::get<2>(result) << ")" << std::endl;
            } else {
                std::cout << "    >> CONTROL: Route setup failed for " << target_node_id << std::endl;
                return std::make_tuple(-1, -1, -1);
            }
            
            // Remove from pending since we're done
            m_pending_route_setups.erase(target_node_id);
        } else {
            // Route setup in progress, drop packet
            std::cout << "    >> CONTROL: Route setup in progress for " << target_node_id << ", dropping packet" << std::endl;
            return std::make_tuple(-1, -1, -1);
        }
    }
    
    return result;
}

void ArbiterSingleForward::SetSingleForwardState(int32_t source_node_id, int32_t target_node_id, int32_t next_node_id, int32_t own_if_id, int32_t next_if_id) {
    NS_ABORT_MSG_IF(next_node_id == -2 || own_if_id == -2 || next_if_id == -2, "Not permitted to set invalid (-2).");
    m_next_hop_list[std::make_pair(source_node_id, target_node_id)] = std::make_tuple(next_node_id, own_if_id, next_if_id);
}

void ArbiterSingleForward::SetRouteSetupCallback(Callback<bool, int32_t, int32_t> callback) {
    m_route_setup_callback = callback;
}

std::string ArbiterSingleForward::StringReprOfForwardingState() {
    std::ostringstream res;
    res << "Single-forward state of node " << m_node_id << std::endl;
    for (const auto& kv : m_next_hop_list) {
        int32_t src = kv.first.first;
        int32_t dst = kv.first.second;
        const auto& tup = kv.second;
        res << "  " << src << " -> " << dst << ": ("
            << std::get<0>(tup) << ", "
            << std::get<1>(tup) << ", "
            << std::get<2>(tup) << ")" << std::endl;
    }
    return res.str();
}

}

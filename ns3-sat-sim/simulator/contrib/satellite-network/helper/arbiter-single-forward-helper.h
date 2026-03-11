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

#ifndef ARBITER_SINGLE_FORWARD_HELPER
#define ARBITER_SINGLE_FORWARD_HELPER

#include "ns3/ipv4-routing-helper.h"
#include "ns3/basic-simulation.h"
#include "ns3/topology-satellite-network.h"
#include "ns3/ipv4-arbiter-routing.h"
#include "ns3/arbiter-single-forward.h"
#include "ns3/abort.h"
#include <limits>

#include "ns3/topology-satellite-network.h"
#include "ns3/openflow-module.h"
#include "ns3/point-to-point-laser-net-device.h"
#include "ns3/point-to-point-laser-channel.h"
#include "ns3/gsl-net-device.h"
#include <queue>
#include <set>
#include <map>

namespace ns3 {

    class ArbiterSingleForwardHelper
    {
    public:
        ArbiterSingleForwardHelper(Ptr<BasicSimulation> basicSimulation, NodeContainer nodes); // legacy constructor
        ArbiterSingleForwardHelper(Ptr<BasicSimulation> basicSimulation, NodeContainer nodes, Ptr<TopologySatelliteNetwork> topology);
    private:
        // Cost function constants
        static constexpr double COST_PARAM_A = 1.0;  // hop weight
        static constexpr double COST_PARAM_B = 5.0;  // ISL utilization weight
        static constexpr double COST_PARAM_GSL = 5.0;  // GSL utilization weight
        static constexpr double MAX_UTILIZATION = 0.99; // threshold for infinite cost
        static constexpr int32_t RELAY_GROUND_STATION_COUNT = 198;
        static constexpr int32_t ANTENNA_COUNT_PER_GS = 8;
        static constexpr int32_t MAX_CONNECTION_PER_SAT = 8;

        std::vector<std::map<std::pair<int32_t, int32_t>, std::tuple<int32_t, int32_t, int32_t>>> InitialEmptyForwardingState();
        void UpdateForwardingState(int64_t t);
        void UpdateReachabilityState(int64_t current_node_id, int64_t target_node_id, bool is_direct_connection);
        void InitializeAccessibleNodes();
        void UpdateAccessibleNodes(int64_t sat_node_id, int64_t relay_gs_node_id);
        std::pair<int64_t, int64_t> FindInterfaceIds(int64_t from_node_id, int64_t to_node_id);
        void SetupGroundStationRoute(uint32_t gs_a, uint32_t gs_b, const std::vector<int32_t>& sat_path);
        void SetupOptimalRoute(uint32_t gs_a, uint32_t gs_b);

        // Optimized routing methods
        void BuildNeighborCache();
        std::pair<std::vector<int32_t>, double> ComputeMultiSourceDijkstra(
            const std::set<int64_t>& source_satellites,
            const std::set<int64_t>& target_satellites
        );
        double CalculateEdgeCost(int32_t from_node, int32_t to_node);
        double CalculateGSLEdgeCost(int32_t from_node_id, int32_t to_node_id);
        double GetLinkUtilization(int32_t from_node, int32_t to_node);
        double GetSatelliteGSLUtilization(int32_t from_node_id, int32_t to_node_id);
        std::vector<std::pair<int32_t, double>> GetNeighborsWithCost(int32_t node_id);
        const std::vector<std::tuple<int32_t, int32_t, int32_t>>& GetNeighbors(int32_t node_id);
        bool IsRouteStillValid(uint32_t gs_a, uint32_t gs_b);
        void ValidateActiveRoutes();
        int32_t FindInterfaceId(Ptr<NetDevice> device);

        // Parameters
        Ptr<BasicSimulation> m_basicSimulation;
        NodeContainer m_nodes;
        int64_t m_dynamicStateUpdateIntervalNs;
        std::vector<Ptr<ArbiterSingleForward>> m_arbiters;

        Ptr<TopologySatelliteNetwork> m_topology; // For convenience, if needed

        // Reachability tracking: maps sat/gs node_id to set of reachable gs/sat nodes
        std::map<int64_t, std::set<int64_t>> m_reachable_nodes;

        // Accessible nodes tracking: maps sat/gs node_id to set of currently accessible gs/sat nodes (directly connected via GSL links)
        std::map<int64_t, std::set<int64_t>> m_accessible_nodes;

        // On-demand flow tracking: GS pairs that have active flows
        std::set<std::pair<uint32_t, uint32_t>> m_active_gs_flows;

        // Optimization caches
        // Neighbor cache: node_id -> list of (neighbor_id, from_interface_id, to_interface_id)
        std::map<int32_t, std::vector<std::tuple<int32_t, int32_t, int32_t>>> m_neighbor_cache;
        bool m_neighbor_cache_built = false;
        
        // Active route path cache: (source_gs, target_gs) -> full mixed path (sat + possible relay GS)
        std::map<std::pair<uint32_t, uint32_t>, std::vector<int32_t>> m_cached_route_paths;

    public:
        // On-demand route setup (called by arbiters via callback)
        bool HandleRouteSetupRequest(int32_t source_node_id, int32_t target_node_id);
        void CollectPathStatistics();

    };

} // namespace ns3

#endif /* ARBITER_SINGLE_FORWARD_HELPER */

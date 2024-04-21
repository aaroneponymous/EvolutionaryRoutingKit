#include <iostream>
#include <vector>
#include "routingkit/osm_simple.h"

using namespace RoutingKit;
using namespace std;

struct SimplifiedTrafficGraph {
    SimpleOSMCarRoutingGraph graph;
    vector<unsigned> traffic_signal_delay;
    vector<bool> is_intersection;  // To mark intersection nodes

    SimplifiedTrafficGraph(const string& pbf_file) {
        graph = simple_load_osm_car_routing_graph_from_pbf(pbf_file);
        traffic_signal_delay.resize(graph.node_count(), 0);  // Initialize with no delay
        is_intersection.resize(graph.node_count(), false);   // Initially, no nodes are marked as intersections
        
        // Determine intersections
        identify_intersections();
    }

    void identify_intersections() {
        cout << "Intersection Nodes:" << endl;
        for (unsigned node = 0; node < graph.node_count(); ++node) {
            unsigned start_arc = graph.first_out[node];
            unsigned end_arc = (node + 1 < graph.node_count()) ? graph.first_out[node + 1] : graph.head.size();
            unsigned outgoing_count = end_arc - start_arc;

            // Consider as intersection if more than two roads connect to this node
            if (outgoing_count > 2) {
                is_intersection[node] = true;
                cout << "Node ID " << node << " is an intersection." << endl;
            }
        }
    }

    void simulate_traffic() {
        // Simple traffic congestion simulation
        for (unsigned i = 0; i < graph.travel_time.size(); ++i) {
            double congestion_factor = 1.0 + (rand() % 5) / 10.0;  // Random increase by up to 50%
            graph.travel_time[i] *= congestion_factor;
        }

        // Simple traffic signal delay simulation but only at intersections
        for (unsigned node = 0; node < graph.node_count(); ++node) {
            if (is_intersection[node]) {
                traffic_signal_delay[node] = rand() % 30;  // Random delay between 0 to 30 seconds at intersections only
            }
        }
    }
};

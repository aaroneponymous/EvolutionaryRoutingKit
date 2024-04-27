#include <ant_colony_system.hpp>
#include </home/aaroneponymous/RoutingKit/include/routingkit/osm_simple.h>
#include <iostream>

using namespace RoutingKit;
using namespace std;

int main()
{
    // Path to your OSM PBF file
    string pbf_file = "../rochester.osm.pbf";

    // Load the graph
    cout << "Loading OSM data..." << endl;
    auto graph = simple_load_osm_car_routing_graph_from_pbf(pbf_file);

    // Convert graph data to the format your ACS expects, if necessary
    // For simplicity, assume your ACS can directly use RoutingKit's graph structure
    cout << "Graph loaded with " << graph.node_count() << " nodes and "
         << graph.arc_count() << " arcs." << endl;

    // Define start and goal nodes
    unsigned start_node = 31258; // Replace with the actual start node ID
    unsigned goal_node = 16076;  // Replace with the actual goal node ID

    // Initialize the Ant Colony System with parameters
    int num_ants = 10;
    double alpha = 1.0;
    double beta = 1.0;
    double decay_rate = 0.05;
    double Q = 100.0;
    int iterations = 50;

    AntColonySystem acs(graph, num_ants, start_node, goal_node, alpha, beta, decay_rate, Q, iterations);

    // Run the optimization process
    cout << "Running Ant Colony Optimization..." << endl;
    acs.run_optimization();

    // Print the results
    acs.print_ant_paths();

    return 0;
}
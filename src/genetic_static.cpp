/**
 * @file genetic_static.cpp
 * @brief Contains the GeneticStatic Class
 */


#include "genetic_static.hpp"
#include "simple_traffic_graph.cpp"
#include <routingkit/osm_graph_builder.h>
#include <routingkit/nested_dissection.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/inverse_vector.h>
#include <iostream>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <chrono>

using namespace RoutingKit;
using namespace std;



void print_chromosome(const std::vector<unsigned> &chromosome)
{
    for (unsigned gene : chromosome)
    {
        std::cout << gene << " -> ";
    }
    std::cout << "End\n";
}

void GeneticStatic::initialize_pop()
{
    population_paths_.clear();
    for (unsigned i = 0; i < population_size_; ++i)
    {
        std::vector<unsigned> path = dijkstra(src_node, dst_node); // Using Dijkstra for initial paths
        if (!path.empty())
        {
            print_chromosome(path);
            population_paths_.push_back(path);
        }
    }
}

std::vector<unsigned> GeneticStatic::dijkstra(unsigned src, unsigned dst)
{
    std::vector<unsigned> dist(graph_->node_count(), std::numeric_limits<unsigned>::max());
    std::vector<int> prev(graph_->node_count(), -1);
    std::priority_queue<DijkstraState, std::vector<DijkstraState>, std::greater<>> pq;

    pq.push({src, 0});
    dist[src] = 0;

    while (!pq.empty())
    {
        auto current = pq.top();
        pq.pop();

        unsigned curr_node = current.node;
        unsigned curr_cost = current.cost;

        if (curr_node == dst)
            break; // Stop if we reach the destination

        if (curr_cost > dist[curr_node])
            continue; // Skip if not the shortest path to curr_node

        unsigned start_arc = graph_->first_out[curr_node];
        unsigned end_arc = graph_->first_out[curr_node + 1];

        for (unsigned i = start_arc; i < end_arc; ++i)
        {
            unsigned next_node = graph_->head[i];
            unsigned weight = graph_->travel_time[i];
            unsigned next_cost = curr_cost + weight;

            if (next_cost < dist[next_node])
            {
                dist[next_node] = next_cost;
                prev[next_node] = curr_node;
                pq.push({next_node, next_cost});
            }
        }
    }

    // Backtrack to find the path
    std::vector<unsigned> path;
    if (dist[dst] == std::numeric_limits<unsigned>::max())
    {
        return path; // return empty path if no path found
    }

    for (int at = dst; at != -1; at = prev[at])
    {
        path.push_back(at);
    }
    std::reverse(path.begin(), path.end());

    return path;
}


// Test Helpers

// 43.15447, -77.60518
// 43.12876, -77.62791

// 31258
// 16076

int main() {

    std::string pbf_file = "../../rochester.osm.pbf";

    SimplifiedTrafficGraph traffic(pbf_file);



/* 
    SimpleOSMCarRoutingGraph test_graph = simple_load_osm_car_routing_graph_from_pbf(
        pbf_file,
        nullptr,
        false,
        false);

    auto graph = std::make_shared<SimpleOSMCarRoutingGraph>(test_graph);

    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();

    GeneticStatic test_algorithm(graph, 1, 0, 0, 0, 31258, 16076);


    // Stop timing
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    std::cout << "Execution time of test_algorithm: " << duration << " milliseconds." << std::endl; */

    return 0;
};


/* #include <routingkit/osm_graph_builder.h>
#include <routingkit/nested_dissection.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/inverse_vector.h>
#include <iostream>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <chrono>

using namespace RoutingKit;
using namespace std;

// Test Helpers
bool is_routing_node(uint64_t osm_node_id, const RoutingKit::TagMap &node_tags)
{
    const char *tag_value = node_tags["highway"];
    return tag_value != nullptr;
}

bool is_way_used_for_routing(uint64_t osm_way_id, const RoutingKit::TagMap &way_tags)
{
    // Include only certain types of ways based on 'highway' tag
    const char *highway = way_tags["highway"];
    return highway != nullptr && (strcmp(highway, "motorway") == 0 || strcmp(highway, "primary") == 0 || strcmp(highway, "secondary") == 0);
}

RoutingKit::OSMWayDirectionCategory way_callback(uint64_t osm_way_id, unsigned routing_way_id, const RoutingKit::TagMap &way_tags)
{
    const char *oneway = way_tags["oneway"];
    if (oneway != nullptr && strcmp(oneway, "yes") == 0)
    {
        return RoutingKit::OSMWayDirectionCategory::only_open_forwards;
    }
    else if (oneway != nullptr && strcmp(oneway, "-1") == 0)
    {
        return RoutingKit::OSMWayDirectionCategory::only_open_backwards;
    }
    return RoutingKit::OSMWayDirectionCategory::open_in_both; // Default case, two-way street
}

// 43.15447, -77.60518
// 43.12876, -77.62791

// 31258
// 16076

int main()
{

    try
    {
        std::string pbf_file = "../../rochester.osm.pbf";

        // Load OSM ID mappings
        auto id_mapping = RoutingKit::load_osm_id_mapping_from_pbf(
            pbf_file,
            is_routing_node,
            is_way_used_for_routing,
            [](const std::string &msg)
            { std::cout << msg << std::endl; }, // Logging function
            false                               // Use this flag as needed, based on your model settings
        );

        // Load the routing graph using the ID mappings
        RoutingKit::OSMRoutingGraph graph = RoutingKit::load_osm_routing_graph_from_pbf(
            pbf_file,
            id_mapping,
            [](uint64_t osm_way_id, unsigned routing_way_id, const RoutingKit::TagMap &way_tags) -> RoutingKit::OSMWayDirectionCategory
            {
                const char *oneway = way_tags["oneway"];
                if (oneway && strcmp(oneway, "yes") == 0)
                {
                    return RoutingKit::OSMWayDirectionCategory::only_open_forwards;
                }
                return RoutingKit::OSMWayDirectionCategory::open_in_both;
            },
            nullptr, // Turn restrictions callback, implement if needed
            [](const std::string &msg)
            { std::cout << msg << std::endl; }, // Logging function
            false                               // Assumes the file is ordered as per the header
        );

        std::cout << "Graph loaded successfully!\n";
        std::cout << "Node count: " << graph.node_count() << ", Arc count: " << graph.arc_count() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
} */


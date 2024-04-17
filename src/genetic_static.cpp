/**
 * @file genetic_static.cpp
 * @brief Contains the GeneticStatic Class
 */

#include "genetic_static.hpp"
#include </home/aaroneponymous/RoutingKit/include/routingkit/osm_simple.h>
#include <vector>
#include <iostream>
#include <cstdlib>
#include <ctime>

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

// 43.15447, -77.60518
// 43.12876, -77.62791

// 31258
// 16076

int main()
{
    std::string pbf_file = "../../rochester.osm.pbf";
    SimpleOSMCarRoutingGraph test_graph = simple_load_osm_car_routing_graph_from_pbf(
        pbf_file,
        nullptr,
        false,
        false);

    auto graph = std::make_shared<SimpleOSMCarRoutingGraph>(test_graph);

    GeneticStatic test_algorithm(graph, 2, 0, 0, 0, 31258, 16076);

    return 0;
}

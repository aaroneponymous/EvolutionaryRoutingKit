#include <ant_colony_system.hpp>
#include </home/aaroneponymous/RoutingKit/include/routingkit/osm_simple.h>
#include <routingkit/geo_position_to_node.h>
#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <iostream>
#include <random>
#include <thread>
#include <tuple>
#include <string>
#include <set>

using namespace RoutingKit;
using namespace std;

// [ ] Accounting for Cul De Sac
// [ ] Remove Loop

void AntColonySystem::get_path(double const src_latitude, double const src_longitude, double const dest_latitude, double const dest_longitude)
{
    // Build the index to quickly map latitudes and longitudes
    GeoPositionToNode map_geo_poisiton(graph_.latitude, graph_.longitude);

    unsigned source_id = map_geo_poisiton.find_nearest_neighbor_within_radius(src_latitude, src_longitude, 1000).id;
    if (source_id == invalid_id)
    {
        std::cout << "No node within 1000m from source position" << std::endl;
        return;
    }

    unsigned destination_id = map_geo_poisiton.find_nearest_neighbor_within_radius(dest_latitude, dest_longitude, 1000).id;
    if (destination_id == invalid_id)
    {
        std::cout << "No node within 1000m from target position" << std::endl;
        return;
    }

    std::cout << "Source Node: " << source_id << ", ";
    std::cout << "Destination Node: " << destination_id << "\n";

    unsigned start_node = source_id;
    unsigned goal_node = destination_id;

    // ACS Algorithm
    unsigned node_count = graph_.node_count();

    const unsigned max_steps =  2000;

    for (int i = 0; i < iterations_; i++)
    {
        for (int j = 0; j < num_ants_; j++)
        {

            Ant ant(j, source_id, goal_node);
            ants_.push_back(ant);

            while (ant.curr_node_ != ant.goal_node_ && ant.steps_ < max_steps)
            {

                /**
                 * @details: choose_node(Ant& ant)
                 *
                 *          1. Gets feasible neighbourhood of the node ant k is in
                 *          2. Gets proabilites on each arc to the neighbour using Eq (1)
                 *          3. Chooses the next node to visit (Exploration vs. Exploitation Balance) Eq (2)
                 *          4. Updates the state of ant and its attributes
                 *          5. Returns the arc index (for pheromone updates)
                 *
                 */


                // Arc ID
                unsigned arc_index = choose_next_arc(ant);

                // Local Update
                double pheromone_trail = pheromone_list_[arc_index];
                pheromone_list_[arc_index] = ((1 - decay_rate_) * pheromone_trail) + (decay_rate_ * initial_pheromones);

                if (distance_global_ != 0 && ant.distance_ > distance_global_)
                    break;
            }

            // print_ant_path(ant);

            if (distance_global_ == 0 || ant.distance_ < distance_global_)
            {
                distance_global_ = ant.distance_;
                global_tour_arcs_ = ant.indices_arcs;
                global_tour_ = ant.path_;
                global_ordered = ant.visited_nodes_;

                // print_global_path();
                // std::cout << std::endl;
            }
        }

        // Iteration Complete: Update Global Pheromone Trail

        for (const auto &index_pheromone : global_tour_arcs_)
        {
            double pheromone_update = (1 - decay_rate_) * pheromone_list_[index_pheromone] + decay_rate_ * (1 / distance_global_);
            pheromone_list_[index_pheromone] = pheromone_update;
        }
    }
}

unsigned AntColonySystem::choose_next_arc(Ant &ant)
{

    std::random_device device;
    std::mt19937 engine(device());
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    // Parameters for Decision
    double q0 = q_;
    double q = distribution(engine); // Generate random number
    double prob_sum = 0;

    // Retrieve Last Arc Index Used if any
    int last_arc_index = ant.indices_arcs.empty() ? -1 : ant.indices_arcs.back();

    // First get all the neighbours
    std::vector<unsigned> neighbourhood;
    std::vector<unsigned> true_neighbours;
    std::vector<unsigned> local_indices_arcs;
    std::vector<double> probabilites;

    // [ ] Finding Set Difference: Nodes That Can be Moved To Based on Turn Restrictions

    // BUG: Indexing (Out of Bounds)
    // last node: ant.curr_node_ + 1 leads to arc count rather than
    // the limit

    // Indices for First Out to Head
    unsigned start_node_idx = graph_.first_out[ant.curr_node_]; // at Node 4 - Index Start = 11
    unsigned end_node_idx = (ant.curr_node_ + 1 == graph_.first_out.size()) ? graph_.head.size() : graph_.first_out[ant.curr_node_ + 1];

    // std::cout << "Start = " << start << " End = " << end << std::endl;

    for (unsigned j = start_node_idx; j < end_node_idx; ++j)
    {

        // Check for forbidden transitions
        if (last_arc_index != -1 && is_transition_forbidden(last_arc_index, j))
        {
            continue; // Skip this neighbour if the transition is forbidden
        }

        unsigned node_id = graph_.head[j];
        neighbourhood.push_back(graph_.head[j]);
        local_indices_arcs.push_back(j);

        // [x] Probabilities Calculation

        double pheromone_amount = pheromone_list_[j];
        double geo_distance = graph_.geo_distance[j];
        double heuristic_ = 1 / geo_distance;

        double probability = std::pow(pheromone_amount, alpha_) * std::pow(heuristic_, beta_);
        probabilites.push_back(probability);
        prob_sum += probability;
    }

    // Normalization Probability
    for (double &prob : probabilites)
    {
        prob /= prob_sum;
    }

    // Decision Making Here: Exploitation (Greedy) or Exploration (Probabilistic)
    unsigned next_node_index = 0;

    if (q <= q0)
    {

        // Exploitation: Choose the best option (greedy)
        // std::cout << "\n"
        //           << "Exploitation:" << std::endl;
        auto it = std::max_element(probabilites.begin(), probabilites.end());
        next_node_index = std::distance(probabilites.begin(), it);
        // std::cout << "Probability Node [" << neighbourhood[next_node_index] << "]: " << probabilites[next_node_index] << std::endl;
    }
    else
    {
        // Exploration: Choose based on probability distribution
        // std::cout << "\n"
        //           << "Exploration:" << std::endl;
        std::discrete_distribution<unsigned> weighted_choice(probabilites.begin(), probabilites.end());
        next_node_index = weighted_choice(engine);
    }

    unsigned next_node = neighbourhood[next_node_index];
    ant.curr_node_ = next_node;
    // remove_loop(ant);
    ant.path_.push_back(next_node);
    ant.steps_++;
    ant.indices_arcs.push_back(local_indices_arcs[next_node_index]);
    if (ant.visited_nodes_.find(next_node) != ant.visited_nodes_.end())
    {
        ant.visited_nodes_.erase(std::prev(ant.visited_nodes_.end()));
    }

    ant.visited_nodes_.insert(next_node);
    ant.distance_ += graph_.geo_distance[local_indices_arcs[next_node_index]];

    // Return the index of the chosen arc, which is required for local pheromone update
    return local_indices_arcs[next_node_index];
}

void AntColonySystem::print_ant_path(Ant &ant)
{
    for (const auto &node : ant.path_)
    {
        std::cout << node << " ";
    }

    std::cout << std::endl;
}

bool AntColonySystem::is_transition_forbidden(unsigned from_arc, unsigned to_arc)
{
    // This method checks the forbidden turn arrays to see if the transition from from_arc to to_arc is not allowed
    for (auto i = 0; i < graph_.forbidden_turn_from_arc.size(); ++i)
    {
        if (graph_.forbidden_turn_from_arc[i] == from_arc && graph_.forbidden_turn_to_arc[i] == to_arc)
        {
            return true;
        }
    }
    return false;
}

#ifdef OSM_CAR_STRUCT_H
struct SimpleOSMCarRoutingGraph
{
    std::vector<unsigned> first_out;
    std::vector<unsigned> head;
    std::vector<unsigned> travel_time;
    std::vector<unsigned> geo_distance;
    std::vector<float> latitude;
    std::vector<float> longitude;
    std::vector<unsigned> forbidden_turn_from_arc;
    std::vector<unsigned> forbidden_turn_to_arc;

    unsigned node_count() const
    {
        return first_out.size() - 1;
    }

    unsigned arc_count() const
    {
        return head.size();
    }
};
#endif // OSM_CAT_STRUCT_H

// #ifdef BUILD_INDIVIDUAL

int main()
{
    // Path to your OSM PBF file
    std::string pbf_file = "../../rochester.osm.pbf";

    // Load the graph
    std::cout << "Loading OSM data..." << std::endl;
    SimpleOSMCarRoutingGraph graph = simple_load_osm_car_routing_graph_from_pbf(pbf_file);

    // Convert graph data to the format your ACS expects, if necessary
    // For simplicity, assume your ACS can directly use RoutingKit's graph structure
    std::cout << "Graph loaded with " << graph.node_count() << " nodes and "
              << graph.arc_count() << " arcs." << std::endl;

    // Geo Positions
    double src_lat = 43.15465;
    double src_long = -77.60473;
    double dst_lat = 43.15164;
    double dst_long = -77.60327;

    // 43.10527,-77.63279
    // 43.10521,-77.62737

    // To get from 21814 to 25879 one needs 144251 milliseconds.
    // This query was answered in 15 microseconds.
    // The path is 21814 21813 21086 16076 4668 24562 21068 23783 20386 22199 22318 21084 23779 23538 25879

    // [x] Create own testing graph

    SimpleOSMCarRoutingGraph test_graph_1;
    //                  Node: 0  1  2  3  4   5  (Arc Counts Last Element)
    test_graph_1.first_out = {0, 2, 5, 8, 11, 13, 8};
    test_graph_1.head =
        {
            1, 4,    // Node 0
            0, 3, 2, // Node 1
            1, 3, 5, // Node 2
            1, 2, 5, // Node 3
            0, 5,    // Node 4
            4, 2, 3  // Node 5
        };

    test_graph_1.geo_distance =
        {
            10, 4,     // Node 0
            10, 30, 2, // Node 1
            2, 5, 10,  // Node 2
            30, 5, 10, // Node 3
            4, 6,      // Node 4
            6, 10, 10  // Node 5
        };

    SimpleOSMCarRoutingGraph test_graph_2;
    // Node: 0   1   2   3   4   5   6   7   (Arc Counts Last Element)
    test_graph_2.first_out = {0, 3, 7, 10, 12, 14, 16, 18, 19};
    test_graph_2.head =
        {
            1, 2, 5,    // Node 0: Connections to nodes 1, 2, 5
            0, 3, 6, 4, // Node 1: Connections to nodes 0, 3, 6, 4
            1, 3, 7,    // Node 2: Connections to nodes 1, 3, 7
            2, 6, 7,    // Node 3: Connections to nodes 2, 6, 7
            5, 1,       // Node 4: Connections to nodes 5, 1
            0, 4,       // Node 5: Connections to nodes 0, 4
            1, 3,       // Node 6: Connections to nodes 1, 3
            2, 3        // Node 7: Connections to nodes 2, 3
        };

    test_graph_2.geo_distance =
        {
            5, 10, 15,  // Distances from Node 0
            5, 7, 3, 8, // Distances from Node 1
            10, 5, 20,  // Distances from Node 2
            5, 12, 2,   // Distances from Node 3
            4, 8,       // Distances from Node 4
            15, 4,      // Distances from Node 5
            3, 12,      // Distances from Node 6
            20, 2       // Distances from Node 7
        };

    // Initialize the Ant Colony System with parameters
    int num_ants = 100;
    double alpha = 0.35;
    double beta = 0.75;
    double decay_rate = 0.65;
    double q = 0.70;
    int iterations = 1000;

    // Define start and goal nodes
    unsigned start_node = 0; // Replace with the actual start node ID
    unsigned goal_node = 3;  // Replace with the actual goal node ID

    // From: 43.10527,-77.63279
    // To:   43.10521,-77.62737
    // To:   43.10479,-77.62605

    // From: 43.23468,-77.54830
    // To:   43.16012,-77.60760

    AntColonySystem acs(graph, num_ants, iterations, alpha, beta, decay_rate, q);
    acs.get_path(43.23468, -77.54830, 43.16012, -77.60760);

    std::cout << "\nPrinting Global Best\n"
              << std::endl;

    acs.print_global_path();

    std::cout << "\n Printing Visited Global \n"
              << std::endl;

    acs.print_global_vist();

    return 0;
}

// #endif // BUILD_INDIVIDUAL

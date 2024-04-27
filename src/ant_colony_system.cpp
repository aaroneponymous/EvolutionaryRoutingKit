

// 43.15465,-77.60473
// 43.15164,-77.60327

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

using namespace RoutingKit;

void AntColonySystem::get_path(double const src_latitude, double const src_longitude,
                               double const dest_latitude, double const dest_longitude)
{
    // Build the index to quickly map latitudes and longitudes
    /* GeoPositionToNode map_geo_poisiton(graph_.latitude, graph_.longitude);

    unsigned source = map_geo_poisiton.find_nearest_neighbor_within_radius(src_latitude, src_longitude, 1000).id;
    if (source == invalid_id)
    {
        std::cout << "No node within 1000m from source position" << std::endl;
        return;
    }

    unsigned destination = map_geo_poisiton.find_nearest_neighbor_within_radius(dest_latitude, dest_longitude, 1000).id;
    if (destination == invalid_id)
    {
        std::cout << "No node within 1000m from target position" << std::endl;
        return;
    }
 */
    // std::cout << "Source Node: " << source << ", ";
    // std::cout << "Destination Node: " << destination << "\n";

    // Algorithm Starts Here (after getting the node IDs)

    // Heurisitically set max steps
    std::vector<unsigned> result_path;
    // const unsigned node_count = graph_.node_count();
    const unsigned max_steps = 20;

    for (int i = 0; i < iterations_; i++)
    {
        std::cout << "\nIteration Number: " << i + 1 << std::endl;
        std::cout << "--------------------" << std::endl;

        for (int j = 0; j < num_ants_; j++)
        {
            // [ ] Assign a thread here if parallelizing

            unsigned start_node = 0;
            unsigned goal_node = 3;

            // BUG: Change source and destination with node ID above
            Ant ant(j, start_node, goal_node);
            // Ant ant(j, source, destination);
            std::cout << "\nAnt Number: " << j + 1 << std::endl;
            std::cout << "-------------\n" << std::endl;
           
            ants_.push_back(ant);
            ant.path_.push_back(ant.curr_node_);
            ant.visited_nodes_.insert(ant.curr_node_);

            // To save the arc indices (arcs on the path)
            std::vector<unsigned> local_arc_index;

            while (ant.curr_node_ != ant.goal_node_ && ant.path_.size() < max_steps)
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


                 std::cout << "Position Current : " << ant.curr_node_ << std::endl;
                 std::cout << "-------------------" << std::endl;

                unsigned arc_index = choose_node(ant);

                // [x] Change ant's current node
                ant.indices_arcs.push_back(arc_index);
                

                // Update Local Pheromone Trail (Local Update Rule)

                // [ ] What if it's the last node and has no neighbours
                // [ ] Return -1 to indicate that (to avoid that we can just
                // [ ] in choose_node method and backtrack accordingly)

                // Debug Update Before and After State
                std::cout << "\nNode Chosen: " << graph_.head[arc_index] << std::endl;
                std::cout << "Pheromone: "
                          << "Before: " << pheromone_list_[arc_index] << " ";

                double pheromone_trail = pheromone_list_[arc_index];
                pheromone_trail = ((1 - decay_rate_) * pheromone_trail) + (decay_rate_ * initial_pheromones);
                pheromone_list_[arc_index] = pheromone_trail;

                std::cout << "After: " << pheromone_list_[arc_index] << std::endl;
                std::cout << "Ant Distance: " << ant.distance_ << "\n" << std::endl;

                // Push Edges in Local Arc Index

                local_arc_index.push_back(arc_index);
            }

            print_ant_path(ant);

            if (distance_global_ == 0 || ant.distance_ < distance_global_)
            {
                distance_global_ = ant.distance_;
                global_tour_arcs_ = local_arc_index;
                result_path = ant.path_;
                global_tour_ = result_path;
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

unsigned AntColonySystem::choose_node(Ant &ant)
{

    std::random_device device;
    std::mt19937 engine(device());
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    // Parameters for Decision
    double q0 = 0.15;
    double q = distribution(engine); // Generate random number

    std::cout << "\n" << "Choosing Node" << std::endl;

    std::cout << "Q0: " << q0 << " Q: " << q << std::endl;

    double prob_sum = 0;

    // First get all the neighbours
    std::vector<unsigned> neighbourhood;
    std::vector<double> probabilites;
    std::vector<unsigned> local_indices_arcs;

    // BUG: Indexing (Out of Bounds)
    // last node: ant.curr_node_ + 1 leads to arc count rather than
    // the limit

    unsigned start = graph_.first_out[ant.curr_node_]; // at Node 4 - Index Start = 11
    unsigned end = 0;
    if (ant.curr_node_ + 1 == graph_.first_out.size() - 1)
    {
        end = graph_.head.size();

    }
    else
    {
        end = graph_.first_out[ant.curr_node_ + 1];
    }

    std::cout << "Start = " << start << " End = " << end << std::endl; 

    // BUG: STUCK IN LOOP
    // Probability Higher for the Path Taken 
    // 3 to 5: Local Update Rule
    // Ant will probably move from 5 to 3 because probability is high

    for (unsigned j = start; j < end; ++j)
    {
        neighbourhood.push_back(graph_.head[j]);
        // [x]: Tracking local indices in the global ant object
        // [x]: Ant arc indicies only saves indices that are needed
        // [x]: to traverse the path
        local_indices_arcs.push_back(j);
    }



    for (const auto &node : neighbourhood)
    {
        std::cout << node << " ";
    }

    std::cout << std::endl;

    // Calculate Probabilities Using Eq 1
    std::cout << "Probability Calculations ";

    for (auto neighbour = 0; neighbour < neighbourhood.size(); ++neighbour)
    {
        if (neighbourhood[neighbour] == ant.goal_node_)
        {
            ant.path_.push_back(neighbourhood[neighbour]);
            ant.distance_ += graph_.geo_distance[local_indices_arcs[neighbour]];
            ant.curr_node_ = neighbourhood[neighbour];
            return local_indices_arcs[neighbour];
        }

        

        // Calculate Probability for this node
        double pheromone_amount = pheromone_list_[local_indices_arcs[neighbour]];
        double geo_distance = graph_.geo_distance[local_indices_arcs[neighbour]];
        double heuristic_ = 0;

        if (geo_distance > 0)
        {
            heuristic_ = 1 / geo_distance;
        }
        else
        {
            // [ ]: Else heuristic 1?
            heuristic_ = 1;
        }

    
        double probability = std::pow(pheromone_amount, alpha_) * std::pow(heuristic_, beta_);
        std::cout << "Node [" << neighbourhood[neighbour] << "]: " << probability << " ";
        probabilites.push_back(probability);
        prob_sum += probability;
    }

    // Normalization Probability
    for (double &prob : probabilites)
    {
        prob /= prob_sum;
    }

    std::cout << std::endl;

    // Print Probabilities

    std::cout << "Normalized Probailities: ";

    for (const auto &prob : probabilites)
    {
        std::cout << prob << " ";
    }

    std::cout << std::endl;


    std::cout << "Neighbours: ";

    for (const auto &node : neighbourhood)
    {
        std::cout << node << " ";
    }

    std::cout << std::endl;


    std::cout << "Arc Indices in Head ";

    for (const auto &arc : local_indices_arcs)
    {
        std::cout << arc << " ";
    }

    std::cout << std::endl;

    // Decision Making Here: Exploitation (Greedy) or Exploration (Probabilistic)
    unsigned next_node_index = 0;

    if (q <= q0)
    {

        // Exploitation: Choose the best option (greedy)
        std::cout << "\n" << "Exploitation:" << std::endl;
        auto it = std::max_element(probabilites.begin(), probabilites.end());
        next_node_index = std::distance(probabilites.begin(), it);
        std::cout << "Probability Node [" << neighbourhood[next_node_index] << "]: " << probabilites[next_node_index] << std::endl;
    }
    else
    {
        // Exploration: Choose based on probability distribution
        std::cout << "\n" << "Exploration:" << std::endl;
        std::discrete_distribution<unsigned> weighted_choice(probabilites.begin(), probabilites.end());
        next_node_index = weighted_choice(engine);
    }

    unsigned next_node = neighbourhood[next_node_index];
    ant.curr_node_ = next_node;
    ant.path_.push_back(next_node);
    ant.visited_nodes_.insert(next_node);
    ant.distance_ += graph_.geo_distance[local_indices_arcs[next_node_index]];


    

    // Return the index of the chosen arc, which is required for local pheromone update
    return local_indices_arcs[next_node_index];
}

void AntColonySystem::print_ant_path(Ant &ant)
{
    for (const auto &node : ant.path_)
    {
        std::cout << "Node: " << node << " ";
    }

    std::cout << std::endl;
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
    /* std::string pbf_file = "../../filtered_rochester.osm.pbf";

    // Load the graph
    std::cout << "Loading OSM data..." << std::endl;
    SimpleOSMCarRoutingGraph graph = simple_load_osm_car_routing_graph_from_pbf(pbf_file);

    // Convert graph data to the format your ACS expects, if necessary
    // For simplicity, assume your ACS can directly use RoutingKit's graph structure
    std::cout << "Graph loaded with " << graph.node_count() << " nodes and "
              << graph.arc_count() << " arcs." << std::endl; */

    // Define start and goal nodes
    unsigned start_node = 31258; // Replace with the actual start node ID
    unsigned goal_node = 16076;  // Replace with the actual goal node ID

    // Geo Positions
    double src_lat = 43.15465;
    double src_long = -77.60473;
    double dst_lat = 43.15164;
    double dst_long = -77.60327;

    // 43.12932, -77.62833
    // 43.12677, -77.6316

    // To get from 21814 to 25879 one needs 144251 milliseconds.
    // This query was answered in 15 microseconds.
    // The path is 21814 21813 21086 16076 4668 24562 21068 23783 20386 22199 22318 21084 23779 23538 25879

    // [x] Create own testing graph

    SimpleOSMCarRoutingGraph test_graph_1;
    //                  Node: 0  1  2  3  4   5
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

    // Initialize the Ant Colony System with parameters
    int num_ants = 20;
    double alpha = 0.5;
    double beta = 0.8;
    double decay_rate = 0.80;
    double Q = 100.0;
    int iterations = 100;

    AntColonySystem acs(test_graph_1, num_ants, alpha, beta, decay_rate, Q, iterations);
    acs.get_path(src_lat, src_long, dst_lat, dst_long);

    std::cout << "\nPrinting Global Best\n" << std::endl;
    acs.print_global_path();


    return 0;
}

// #endif // BUILD_INDIVIDUAL

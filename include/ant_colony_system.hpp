/**
 * @file: ant_colony_system.hpp
 * @brief: Ant Colony System Class
 */

#ifndef ANT_COLONY_SYSTEM_H
#define ANT_COLONY_SYSTEM_H

#include </home/aaroneponymous/RoutingKit/include/routingkit/osm_simple.h>
#include <vector>
#include <unordered_set>
#include <iostream>
#include <iomanip>

using namespace RoutingKit;

/**
 * @brief:
 *
 */

struct Ant
{
    unsigned ant_id_;
    unsigned curr_node_; // or start/source node
    unsigned goal_node_; // or destination node
    unsigned distance_ = 0;
    double time_taken_ = 0;
    bool reached_ = false;
    bool elite_ = false;

    std::vector<unsigned> path_;                 // Path Taken
    std::vector<unsigned> indices_arcs;          // Arc Indices in the Pheromone Matrix
    std::unordered_set<unsigned> visited_nodes_; // Set of Visited Nodes

    Ant(int id, unsigned start, unsigned goal)
        : ant_id_(id), curr_node_(start), goal_node_(goal) {
            path_.push_back(start);
            visited_nodes_.insert(start);
        }
};

/**
 * @brief
 *
 *
 */

// [x] : For a better intital_pheromones heuristic
// [x] : Can construct an initial shortest path tour between all nodes
// [x] : And Calculate it with 1 / Lnn (length of that path)
constexpr double initial_pheromones = 0.5;

class AntColonySystem
{
private:
    int num_ants_;
    int iterations_;
    double alpha_;      // pheromone_coefficient
    double beta_;       // heuristic_coefficient
    double decay_rate_; // (rho) decay rate of pheromones
    double Q_;          // constant multiplication factor for the cost/reward function
    unsigned distance_global_ = 0;

    std::vector<double> pheromone_list_; // Using a list to mimic graph_.head vector
    std::vector<Ant> ants_;
    std::vector<unsigned> global_tour_; //  Global Best Tour
    std::vector<unsigned> global_tour_arcs_;
    SimpleOSMCarRoutingGraph graph_;

public:
    AntColonySystem(SimpleOSMCarRoutingGraph &graph, int n_ants, int iterations, double alpha, double beta, double decay_rate, double Q)
        : graph_(graph), num_ants_(n_ants), iterations_(iterations), alpha_(alpha), beta_(beta), decay_rate_(decay_rate), Q_(Q)
    {
        pheromone_list_.resize(graph_.head.size());
        std::fill(pheromone_list_.begin(), pheromone_list_.end(), initial_pheromones);
    }

    // Change all parameters
    void set_parameters(double alpha, double beta, double decay_rate, double Q)
    {
        alpha_ = alpha;
        beta_ = beta;
        decay_rate_ = decay_rate;
        Q_ = Q;
    }

    // Path Planning Methods
    // [ ] Parameter type unsigned const for testing purposes
    void get_path(unsigned const start_id, unsigned const end_id);

    /* void get_path(double const src_latitude, double const src_long,
                  double const dst_latitdue, double const dest_long); */

    // Return arc index
    unsigned choose_node(Ant &ant);

    // Probability & Related Computational Functions

    std::vector<double> get_probabilities(Ant &ant); // Probability of moving from node i to neighbours

    /**
     * @brief Removes loops in path
     * @param ant Ant in whose path the loops are to be removed
     * @return void
     * @details Removes loops in path of an ant only when a point is revisted.
     */
    static void remove_loop(Ant &ant);

    void print_ant_path(Ant& ant);

    void print_global_path()
    {
        for (const auto& node : global_tour_)
        {
            std::cout << "Node: " << node << " ";
        }

        std::cout << std::endl;

        std::cout << "Distance: " << distance_global_ << std::endl;
    }

    // Accessors & Mutators

    double get_alpha() const { return alpha_; }
    double get_beta() const { return beta_; }
    double get_decay_rate() const { return decay_rate_; }
    double get_Q() const { return Q_; }

    void set_alpha(double alpha) { alpha_ = alpha; }
    void set_beta(double beta) { beta_ = beta; }
    void set_decay_rate(double decay_rate) { decay_rate_ = decay_rate; }
    void set_Q(double Q) { Q_ = Q; }
};

/**
 * @brief: Ant-Colony System Implementation Details
 *
 *         1. Tour Construction of ACS
 *            - Use the Initial Parameter and Compute Values
 *
 *            @parameter: 1. k (number of ants)
 *
 *            - An ant k in node i chooses the next node j with a probability defined
 *              by the "Random Proportional Rule" | P of k moving from i to j:
 *            - If j belongs to N(k)(i) (feasible neighborhood)
 *
 *            @equation:
 *
 *            @parameters:
 *                        2.  history_coefficient (alpha)   - Controls the influence of the pheromone train
 *                                                            in decision making
 *
 *                        3.  heurisitic_coefficient (beta) - Determines the importance of the heuristic
 *                                                            information in the decision making
 *
 *                        4.  pheromone_matrix              - Matrix to record pheromones deposit
 *                                                          - T (ij): Amount of pheromones deposited for a state
 *                                                            transition from i to j
 *                                                          - n (ij): Desirability of state transition i to j
 *                                                            (a priori knowledge, typically 1/d(ij) d: distance
 *                                                            from i to j)
 *                        5.  cost_matrix
 *
 *              @decision:
 *                        In ACS the Random Proportional Rule with probability q0 E [0, 1] for the
 *                        next city visit is defined as:
 *
 *                        - j =
 *
 *                        - Otherwise J, where J is a random variable given by EQ (1)
 *
 *          2. Global Pheromone Trail Update
 *
 *             - After each iteration, the shortest tour (global-best tour) of this iteration is determined
 *               and arcs belonging to this tour receive "extra" pheromone, so only the global-best tour
 *               allow ants to add pheromone after each iteration by the global updating rule:
 *
 *             - @function: T(ij) (t + 1) = ( 1 - rho)Tij(t) + rho delta T(global)(ij)[t] with all (i,j) E global-best tour
 *
 *
 *
 *             - @note: Pheromone Trail Update Rule is only applied to the arcs of the global-best tour,
 *                      not to all arcs like in AS
 *
 *             @parameters:
 *
 *                         1.
 *
 *
 *          3. Local Pheromone Trail Update
 *
 *
 *             - In addition the ants use a local update rule that is immediately applied after visiting an arc during the construction
 *               in ACS
 *
 *             - @function: T(ij) = (1 - decay) x Tij + decay x Tij
 *
 */

#endif // ANT_COLONY_SYSTEM_H
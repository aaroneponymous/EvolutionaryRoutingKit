/**
 * @file: ant_colony_system.hpp
 * @brief: Ant Colony System Class
 */

#ifndef ANT_COLONY_SYSTEM_H
#define ANT_COLONY_SYSTEM_H

#include <vector>
#include <routingkit/osm_simple.h>

using namespace RoutingKit;

struct Ant
{
    /**
     * @brief Constructor to create a new ant
     * @param start Start Node of ant
     * @param id    Ant ID
     * @return no return value
     */

    Ant(const unsigned start, const int id = 0)
        : id_(id), current_node_(start), previous_node_(-1) {}

    bool found_goal_ = false;
    int id_;
    int steps_ = 0;
    unsigned current_node_;
    unsigned previous_node_;
    std::vector<unsigned> path_;
};

/**
 * @brief
 */

class AntColonySystem
{
private:
    int num_ants_;
    double alpha_;      // history_coefficient
    double beta_;       // heuristic_coefficient
    double decay_rate_; // decay rate of pheromones
    double Q_;          // total amount of pheromone left on the trail be each ant
    int iterations_;
    std::vector<std::vector<double>> pheromone_matrix_;
    // std::vector<std::vector<double>> pheromone_matrix_global_; // [ ]: Might help in Parallel
    std::vector<std::vector<double>> heuristic_matrix_;
    std::vector<Ant> ants_;
    SimpleOSMCarRoutingGraph graph_;

public:
    // Constructor
    AntColonySystem(SimpleOSMCarRoutingGraph &graph, int n_ants, double alpha, double beta, double decay_rate, double Q, int iterations)
        : graph_(graph), num_ants_(n_ants), alpha_(alpha), beta_(beta), decay_rate_(decay_rate), Q_(Q), iterations_(iterations)
    {
        initialize_matrices();
    }

    // Set Parameters dynamically
    void set_parameters(int n_ants, double alpha, double beta, double decay_rate, double Q, int iterations)
    {
        num_ants_ = n_ants;
        alpha_ = alpha;
        beta_ = beta;
        decay_rate_ = decay_rate;
        Q_ = Q;
        iterations_ = iterations;

        // Re-initialize matrices and other resources if necessary (Do we need to)
        // initialize_matrices();
    };

    // Initialize pheromone and heuristic matrices
    void initialize_matrices(const std::vector<std::vector<unsigned>> &geo_distance);

    // Initialize pheromone and heuristic matrices
    void initialize_matrices();

    // Main ACO process
    void run_optimization();

    // Generate a solution for one ant
    std::vector<unsigned> generate_solution(Ant &ant);

    // Update pheromone matrix after each ant completes its tour
    void update_pheromones_local();

    // Update pheromone matrix globally after all ants have completed their tours
    void update_pheromones_global();

    // Evaporate pheromones to simulate natural decay over time
    void evaporate_pheromones();

    // Choose the next node for an ant based on probability distribution
    unsigned choose_next_node(const Ant &ant, const std::vector<bool> &visited);

    // Calculate the probability of moving to the next node
    double calculate_transition_probability(int from, int to, const std::vector<bool> &visited);

    // Calculate the total cost of a tour
    double calculate_tour_cost(const std::vector<unsigned> &tour);
};

#endif // ANT_COLONY_SYSTEM_H

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
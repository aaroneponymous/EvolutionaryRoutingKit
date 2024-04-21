#include <ant_colony_system.hpp>


void AntColonySystem::initialize_matrices() {
    unsigned node_count = graph_.node_count();
    pheromone_matrix_.resize(node_count, std::vector<double>(node_count, 0.5)); // Initial value can be a small constant
    heuristic_matrix_.resize(node_count, std::vector<double>(node_count));

    for (unsigned i = 0; i < node_count; ++i) {
        unsigned from_index = graph_.first_out[i];
        unsigned to_index = graph_.first_out[i + 1];
        for (unsigned arc = from_index; arc < to_index; ++arc) {
            unsigned j = graph_.head[arc];
            double distance = graph_.geo_distance[arc];
            if (distance > 0)
                heuristic_matrix_[i][j] = 1.0 / distance;
            else
                heuristic_matrix_[i][j] = 0; // Prevent division by zero
        }
    }
}

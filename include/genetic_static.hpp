#ifndef GENETIC_STATIC_H
#define GENETIC_STATIC_H

#include "genetic_abstract.hpp"
#include <vector>
#include <limits>
#include <queue>
#include <limits>

struct DijkstraState
{
    unsigned node;
    unsigned cost;
    bool operator>(const DijkstraState &other) const
    {
        return cost > other.cost;
    }
};

class GeneticStatic : public GeneticAbstract
{
private:
    std::vector<unsigned> chromosome_path_;
    std::vector<std::vector<unsigned>> population_paths_;
    std::vector<double> fitness_val_;
    unsigned population_size_;
    unsigned generations_;
    unsigned curr_gen_ = 0;
    double mutation_rate_;
    double crossover_rate_;
    unsigned src_node;
    unsigned dst_node;

    // Additional [Might/Might Not Use Them]
    size_t path_limit_ = std::numeric_limits<size_t>::max();
    double coefficient_;
    bool path_found_; // Use in termination condition
    bool shorten_chromosome_;

public:
    // Add a parameter for the graph in the constructor
    GeneticStatic(std::shared_ptr<SimpleOSMCarRoutingGraph> graph, unsigned pop_size, unsigned gen, double mut_rate, double cross_rate,
                  unsigned src, unsigned dst)
        : GeneticAbstract(graph), // Pass the graph to the base class constructor
          population_size_(pop_size), generations_(gen), mutation_rate_(mut_rate), crossover_rate_(cross_rate),
          src_node(src), dst_node(dst), path_found_(false)
    {
        initialize_pop();
    }

    ~GeneticStatic() noexcept override {}

    void initialize_pop();

    std::vector<unsigned> dijkstra(unsigned src, unsigned dst);

    bool should_terminate() const override
    {
        return path_found_ || curr_gen_ >= generations_;
    }
};



#endif // GENETIC_STATIC_H
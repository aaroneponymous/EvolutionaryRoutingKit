#ifndef GENETIC_STATIC_H
#define GENETIC_STATIC_H

#include "genetic_abstract.hpp"
#include <vector>
#include <limits>

typedef std::vector<unsigned> chromosome_path_;

class GeneticStatic : public GeneticAbstract
{
private:
    std::vector<chromosome_path_> population_paths_;
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
    GeneticStatic(std::shared_ptr<OSMRoutingGraph> graph, unsigned pop_size, unsigned gen, double mut_rate, double cross_rate,
                  unsigned src, unsigned dst)
        : GeneticAbstract(graph), // Pass the graph to the base class constructor
          population_size_(pop_size), generations_(gen), mutation_rate_(mut_rate), crossover_rate_(cross_rate),
          src_node(src), dst_node(dst), path_found_(false)
    {
        initialize_pop();
    }

    ~GeneticStatic() noexcept override {}

    void initialize_pop() override
    {
        // Implement initialization logic
    }

    void evaluate_fitness() override
    {
        // Implement fitness evaluation logic
    }

    void selection() override
    {
        // Implement selection logic
    }

    void crossover() override
    {
        // Implement crossover logic
    }

    void mutation() override
    {
        // Implement mutation logic
    }

    void run_ga() override
    {
        // Implement the main run loop of the genetic algorithm
    }

    bool should_terminate() const override
    {
        return path_found_ || curr_gen_ >= generations_;
    }
};

#endif // GENETIC_STATIC_H
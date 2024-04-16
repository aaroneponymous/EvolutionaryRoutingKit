#ifndef GENETIC_STATIC_H
#define GENETIC_STATIC_H

#include "genetic_abstract.hpp"
#include <vector>
#include <limits>

typedef std::vector<unsigned> chromosome_path_;

class GeneticStatic : public GeneticAbstract {
private:
    std::vector<chromosome_path_> population_paths_;
    std::vector<double> fitness_val_;
    unsigned population_size_;
    unsigned generations_;
    double mutation_rate_;
    double crossover_rate_;
    unsigned src_node;
    unsigned dst_node;

    // Additional [Might/Might Not Use Them]
    size_t path_limit_ = std::numeric_limits<size_t>::max();
    double coefficient_;
    bool path_found_;           // Use in termination condition
    bool shorten_chromosome_;



public:

    GeneticStatic(unsigned pop_size, unsigned gen, double mut_rate, double cross_rate, 
                  unsigned src, unsigned dst) : population_size_(pop_size), generations_(gen),
                  mutation_rate_(mut_rate), crossover_rate_(cross_rate), src_node(src), dst_node(dst),
                  path_found_(false)
    {
        // Initialize Population Here

    }


    ~GeneticStatic() noexcept override {}

};

#endif // GENETIC_STATIC_H
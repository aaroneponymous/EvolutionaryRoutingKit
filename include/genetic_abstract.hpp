/**
 * @file    genetic_abstract.hpp
 * @brief   Contains the Abstract Genetic Algorithm (GA) Class for concrete implementations
 *          of different variants
*/

#ifndef GENETIC_ABSTRACT_H
#define GENETIC_ABSTRACT_H

#include <routingkit/osm_graph_builder.h>
#include <vector>
#include <memory>

using namespace RoutingKit;

/**
 * @brief Class for the abstract implementation of GA
*/

class GeneticAbstract {
protected:  
    // Shared Graph Data Structure
    std::shared_ptr<OSMRoutingGraph> graph_; // [ ]: Use std::atomic<shared_ptr> to prevent data race
public:
    GeneticAbstract(std::shared_ptr<OSMRoutingGraph> graph) : graph_(graph) {}
    
    virtual ~GeneticAbstract() noexcept {}

    // Initialize the Population
    virtual void initialize_pop() = 0;

    // Fitness Evaluation for individual in population
    virtual void evaluate_fitness() = 0;

    // Selection
    virtual void selection() = 0;

    // Crossover
    virtual void crossover() = 0;

    // Mutation
    virtual void mutation() = 0;

    // Run GA
    virtual void run_ga() = 0;

    // Termination Condition Status
    virtual bool should_terminate() const = 0;

};


#endif // GENETIC_ABSTRACT_H
/**
 * @file    genetic_abstract.hpp
 * @brief   Contains the Abstract Genetic Algorithm (GA) Class for concrete implementations
 *          of different variants
*/

#ifndef GENETIC_ABSTRACT_H
#define GENETIC_ABSTRACT_H

#include </home/aaroneponymous/RoutingKit/include/routingkit/osm_simple.h>
#include <vector>
#include <memory>

using namespace RoutingKit;

/**
 * @brief Class for the abstract implementation of GA
*/

class GeneticAbstract {
protected:  
    // Shared Graph Data Structure
    std::shared_ptr<SimpleOSMCarRoutingGraph> graph_; // [ ]: Use std::atomic<shared_ptr> to prevent data race
public:
    GeneticAbstract(std::shared_ptr<SimpleOSMCarRoutingGraph> graph) : graph_(graph) {}
    
    virtual ~GeneticAbstract() noexcept {}

    // Initialize the Population
    virtual void initialize_pop() = 0;



    // Termination Condition Status
    virtual bool should_terminate() const = 0;

    // Utility Debugging Functions

};


#endif // GENETIC_ABSTRACT_H
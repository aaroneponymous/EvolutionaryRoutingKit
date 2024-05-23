/**
 * @file    abstract_genetic.hpp
 * @brief   Contains the Abstract Genetic Algorithm (GA) Class
*/

#ifndef ABSTRACT_GENETIC_H
#define ABSTRACT_GENETIC_H

/**
 * @brief Templated Abstract Genetic Algorithm Class
 * 
 * @tparam Container  The container type for the population (e.g., std::vector).
 * @tparam Chromosome The type representing a chromosome.
 * @tparam FitnessScore The type representing the fitness score.
*/
template<template<typename> class Container, typename Chromosome, typename FitnessScore>
class AbstractGenetic {
protected:
    using Population = Container<Chromosome>;
    Population population_;

public:
    virtual ~AbstractGenetic() = default;

    /**
     * @brief Initialize the population.
     * 
     * @tparam Args Variadic template arguments.
     * @param args Arguments for population initialization.
     */
    template<typename... Args> 
    virtual void initialize_population(Args&&... args) = 0;

    /**
     * @brief Evaluate the fitness of the population.
     * 
     * @tparam Args Variadic template arguments.
     * @param args Arguments for fitness evaluation.
     * @return FitnessScore The evaluated fitness score.
     */
    template<typename... Args>
    virtual FitnessScore evaluate_fitness(Args&&... args) = 0;

    /**
     * @brief Select individuals from the population.
     * 
     * @tparam Args Variadic template arguments.
     * @param args Arguments for selection.
     */
    template<typename... Args>
    virtual void selection(Args&&... args) = 0;

    /**
     * @brief Perform crossover on the population.
     * 
     * @tparam Args Variadic template arguments.
     * @param args Arguments for crossover.
     */
    template<typename... Args>
    virtual void crossover(Args&&... args) = 0;

    /**
     * @brief Mutate individuals in the population.
     * 
     * @tparam Args Variadic template arguments.
     * @param args Arguments for mutation.
     */
    template<typename... Args>
    virtual void mutation(Args&&... args) = 0;

    /**
     * @brief Determine whether the algorithm should terminate.
     * 
     * @tparam Args Variadic template arguments.
     * @param args Arguments for termination criteria.
     * @return bool Whether the termination criteria are met.
     */
    template<typename... Args>
    virtual bool terminate(Args&&... args) = 0;
};

#endif // ABSTRACT_GENETIC_H

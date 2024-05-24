/**
 * @file    chromosome.hpp
 * @brief   Contains Template for Chromosome used in GA
*/

#ifndef CHROMOSOME_HPP
#define CHROMOSOME_HPP



namespace GENETIC {

    /**
     * @brief: 1. Variadic Template
     *         2. Tuple for Gene Storage
     *   
    */


   /**
    * @typedef: Chromosome Struct: CHR
    * @params:  1. Container: Specifies Generic Container for e.g. vector<T>, tuple<T>
    *           2. Params:    Generic Parameter Pack
    *   
    * @brief:   Allows for flexibility in terms of defining the Gene with a 
    *           Templated Parameter Pack (number and variation in the type 
    *           of parameters) based on the problem the Genetic Algorithm 
    *           is being used to solve
   */

  template<typename Container, typename... Params>
  using CHR = Container<Params...>;

  







}



#endif CHROMOSOME_HPP
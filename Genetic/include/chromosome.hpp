/**
 * @file    chromosome.hpp
 * @brief   Contains Template for Chromosome used in GA
*/

#ifndef CHROMOSOME_HPP
#define CHROMOSOME_HPP

#include <memory>

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


    // Trait to check if a type is a container
    template<typename T, typename _ = void>
    struct is_container : std::false_type {};

    template<typename... Ts>
    struct is_container_helper {};

    template<typename T>
    struct is_container<T, std::conditional_t<
        false,
        is_container_helper<
            typename T::value_type,
            typename T::size_type,
            typename T::allocator_type,
            typename T::iterator,
            typename T::const_iterator,
            decltype(std::declval<T>().size()),
            decltype(std::declval<T>().begin()),
            decltype(std::declval<T>().end())
        >,
        void
    >> : std::true_type {};



    // Primary template for a single parameter
    template<typename Param>
    struct CHR {
        using type = Param;
        std::unique_ptr<Param> value;

        // Constructor
        explicit CHR(const Param& param) : value(std::make_unique<Param>(param)) {}

        // Copy constructor
        CHR(const CHR& other) : value(std::make_unique<Param>(*other.value)) {}

        // Move constructor
        CHR(CHR&& other) noexcept : value(std::move(other.value)) {}

        // Copy assignment operator
        CHR& operator=(const CHR& other) {
            if (this != &other) {
                value = std::make_unique<Param>(*other.value);
            }
            return *this;
        }

        // Move assignment operator
        CHR& operator=(CHR&& other) noexcept {
            if (this != &other) {
                value = std::move(other.value);
            }
            return *this;
        }

        // Destructor
        ~CHR() = default;


        // Equality Operator
        bool operator==(const CHR& other) const {
            if constexpr (is_container<Param>::value) {
                return *value == *other.value;
            }
            else {
                return *value == *other.value;
            }
        }

        // Inequality operator
        bool operator!=(const CHR& other) const {
            return !(*this == other);
        }



    };

}



#endif CHROMOSOME_HPP
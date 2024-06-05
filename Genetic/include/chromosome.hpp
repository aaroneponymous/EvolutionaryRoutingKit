/**
 * @file    chromosome.hpp
 * @brief   Contains Template for Chromosome used in GA
 *
 */

#ifndef CHROMOSOME_HPP
#define CHROMOSOME_HPP

#include <memory>
#include <vector>
#include <tuple>
#include <type_traits>
#include <iostream>
#include <utility>

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

// [x]: Incorporate SFINAE to detect and handle different container types

// [ ]: User Type Objects Not Supported

// Check if a type is a container
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
    >, void >> : std::true_type {};

// Helper function to print tuple elements
template<typename Tuple, std::size_t... Is>
void print_tuple_impl(const Tuple& t, std::index_sequence<Is...>) {
    ((std::cout << (Is == 0 ? "" : ", ") << std::get<Is>(t)), ...) << std::endl;
}

template<typename... Args>
void print_tuple(const std::tuple<Args...>& t) {
    print_tuple_impl(t, std::index_sequence_for<Args...>{});
}

// Check if a type is a tuple
template<typename T>
struct is_tuple : std::false_type {};

template<typename... Args>
struct is_tuple<std::tuple<Args...>> : std::true_type {};

namespace GENETIC {

    template<typename Param, typename T>
    class Chromosome {
        static_assert(is_container<Param>::value || is_tuple<Param>::value, "Param must be a std::vector or std::tuple");
        static_assert(std::is_arithmetic<T>::value, "T must be an arithmetic type");
    private:
        using type = Param;
        Param genes_;                   // represents the estimated parameter(s)
        Param result_;                  // result after application of chromosome objective function(s)
        std::string encoded_chromo;     // string of bits representing chromosome

        T sum_fitness_;                 // total sum of objective function(s) results
        int chromo_size_;               // total no. of bits in encoded_chromo
        int num_generation_;            // no. of generation

    public:
        



    };
    

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

        // Equality operator
        bool operator==(const CHR& other) const {
            return *value == *other.value;
        }

        // Inequality operator
        bool operator!=(const CHR& other) const {
            return !(*this == other);
        }

        // Print function for single parameter
        void print() const {
            print_impl(*value);
        }

    private:
        // Generic print implementation
        template<typename T>
        std::enable_if_t<!is_container<T>::value && !is_tuple<T>::value> print_impl(const T& param) const {
            std::cout << param << std::endl;
        }

        template<typename T>
        std::enable_if_t<is_container<T>::value && !is_tuple<T>::value> print_impl(const T& container) const {
            for (const auto& elem : container) {
                std::cout << elem << " ";
            }
            std::cout << std::endl;
        }

        template<typename T>
        std::enable_if_t<is_tuple<T>::value> print_impl(const T& t) const {
            print_tuple(t);
        }
    };

} // namespace GENETIC

#endif // CHROMOSOME_HPP
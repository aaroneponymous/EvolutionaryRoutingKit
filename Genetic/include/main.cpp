#include "chromosome.hpp"
#include <iostream>
#include <vector>
#include <tuple>

int main() {
    // Create instances of CHR with integer parameters
    GENETIC::CHR<int> chrSingle1(42);
    GENETIC::CHR<int> chrSingle2(42);
    GENETIC::CHR<int> chrSingle3(43);

    // Print values
    chrSingle1.print(); // Output: 42
    chrSingle2.print(); // Output: 42
    chrSingle3.print(); // Output: 43

    // Compare instances
    if (chrSingle1 == chrSingle2) {
        std::cout << "chrSingle1 and chrSingle2 are equal." << std::endl; // This will be printed
    }

    if (chrSingle1 != chrSingle3) {
        std::cout << "chrSingle1 and chrSingle3 are not equal." << std::endl; // This will be printed
    }

    // Create instances of CHR with vector parameters
    GENETIC::CHR<std::vector<int>> chrVec1(std::vector<int>{1, 2, 3});
    GENETIC::CHR<std::vector<int>> chrVec2(std::vector<int>{1, 2, 3});
    GENETIC::CHR<std::vector<int>> chrVec3(std::vector<int>{1, 2, 4});

    // Print values
    chrVec1.print(); // Output: 1 2 3
    chrVec2.print(); // Output: 1 2 3
    chrVec3.print(); // Output: 1 2 4

    // Compare instances
    if (chrVec1 == chrVec2) {
        std::cout << "chrVec1 and chrVec2 are equal." << std::endl; // This will be printed
    }

    if (chrVec1 != chrVec3) {
        std::cout << "chrVec1 and chrVec3 are not equal." << std::endl; // This will be printed
    }

    // Create instances of CHR with tuple parameters
    auto tuple1 = std::make_tuple(1, 2.0, 'a');
    auto tuple2 = std::make_tuple(1, 2.0, 'a');
    auto tuple3 = std::make_tuple(2, 3.0, 'b');

    GENETIC::CHR<std::tuple<int, double, char>> chrTuple1(tuple1);
    GENETIC::CHR<std::tuple<int, double, char>> chrTuple2(tuple2);
    GENETIC::CHR<std::tuple<int, double, char>> chrTuple3(tuple3);

    // Print values
    chrTuple1.print(); // Output: 1, 2.0, a
    chrTuple2.print(); // Output: 1, 2.0, a
    chrTuple3.print(); // Output: 2, 3.0, b

    // Compare instances
    if (chrTuple1 == chrTuple2) {
        std::cout << "chrTuple1 and chrTuple2 are equal." << std::endl; // This will be printed
    }

    if (chrTuple1 != chrTuple3) {
        std::cout << "chrTuple1 and chrTuple3 are not equal." << std::endl; // This will be printed
    }

    return 0;
}

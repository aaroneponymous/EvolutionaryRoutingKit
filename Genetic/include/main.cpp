#include "chromosome.hpp"
#include <iostream>
#include <tuple>
#include <vector>



int main()
{

    // Instance of CHR with single parameters

    GENETIC::CHR<int> single_int1(32);
    GENETIC::CHR<int> single_int2(32);
    GENETIC::CHR<int> single_int3(1);


    single_int1.print();
    single_int2.print();
    single_int3.print();


    // Instance of Tuple Multiple Param
    auto tuple_1 = std::make_tuple(1, 2.0, 'a');
    auto tuple_2 = std::make_tuple(1, 2.0, 'a');
    auto tuple_3 = std::make_tuple(2, 3.0, 'a');

    GENETIC::CHR<std::tuple<int, double, char>> multi_tuple1(tuple_1);
    GENETIC::CHR<std::tuple<int, double, char>> multi_tuple2(tuple_2);
    GENETIC::CHR<std::tuple<int, double, char>> multi_tuple3(tuple_3);


    std::cout << "\nPrinting Tuple-Based Chromosome" << std::endl;
    multi_tuple1.print();
    multi_tuple2.print();
    multi_tuple3.print();


    std::cout << "\nTesting Operators for Tuple-Based Chromosome" << std::endl;
    if (multi_tuple1 == multi_tuple2) {
        std::cout << "Tuple 1 and Tuple 2 are equal" << std::endl;
    }

    if (multi_tuple1 != multi_tuple3) {
        std::cout << "Tuple 1 and Tuple 3 are not equal" << std::endl;

    }


    return 0;

}

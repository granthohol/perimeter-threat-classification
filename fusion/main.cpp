#include <iostream>
#include "fusion.hpp"

int main(int argc, char** argv){

    std::cout << "[perimeter-threat-classification] Starting pipeline...\n";
    fusion::runPipeline();
    std::cout << "[perimeter-threat-classification] Pipeline finished.\n";
    return 0;
}
#include "matplotlibcpp.h"
#include "algorithmmng.h"
#include "formation.hpp"

namespace plt = matplotlibcpp;

int main(int argc, char const *argv[]){
    printf("Fuck onemore run!!!!!!!!!!!!!!!!!!!!!!\n");

    AlgorithmMng am;
    // am.path = "./dac_data";
    am.start();

    
    am.stop();
    printf("SUCCESS finished all jobs\n");
    return 0;
}



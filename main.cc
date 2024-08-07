#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <string>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
// #include <winsock2.h>
// #include <ws2tcpip.h>             // && 临时注释 linux上需要修改此项为兼容的库
// #include <sys/socket.h>           // && 临时注释
// #include <arpa/inet.h>
// #include "covering_algorithm/formation.hpp"
// #include "covering_algorithm/planning.hpp"
// #pragma comment(lib, "Ws2_32.lib")
#include "algorithmmng.h"


int main(int argc, char const *argv[]){
    
    AlgorithmMng am;
    printf("Fuck onemore run!!!!!!!!!!!!!!!!!!!!!!\n");
    am.start();
    am.stop();
    printf("SUCCESS finished all jobs\n");
    return 0;
}


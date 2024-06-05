#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include "formation.hpp"

// Helper function to split a string by a delimiter and return a vector of strings
std::vector<std::string> split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

std::string map(int num)
{
    if (num == 0) return("datax.csv");
    if (num == 1) return("datay.csv");
    if (num == 2) return("dataz.csv");
} 


int main() {
    std::string filename_base = "E:\\Project_C\\Yielded_240605\\";

    for (size_t i = 0; i < 3; i++)
    {
        std::string filename = filename_base + map(i);

        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open the file " << filename << std::endl;
            return 1;
        }

        std::vector<std::vector<std::string>> data;     // 二维vector
        std::string line;

        while (std::getline(file, line)) {
            std::vector<std::string> row = split(line, ',');
            data.push_back(row);
        }

        file.close();
    }







    // Print the CSV file content





    // for (const auto& row : data) {
    //     for (const auto& col : row) {
    //         std::cout << col << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // return 0;
}
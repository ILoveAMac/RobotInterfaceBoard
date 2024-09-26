//
// Created by Wihan on 2024-09-08.
//

#ifndef MODELLOADINGHELPER_H
#define MODELLOADINGHELPER_H

#include <vector>
#include <string>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "cereal/types/vector.hpp"
#include "cereal/archives/binary.hpp"

class ModelLoadingHelper {
public:
    // Constructor takes a file path to the directory containing model parameters
    explicit ModelLoadingHelper(const std::string &filePath);

    // Free any memory allocated if needed
    ~ModelLoadingHelper();

    // Function to load 4D vector of floats, so a conv2d layer
    std::vector<std::vector<std::vector<std::vector<float>>>> loadConv4D(const std::string &key) const;

    // Function to load 2D vector of floats, used for fully connected layers
    std::vector<std::vector<float>> loadFCL(const std::string &key) const;

    // Function to load 1D vector, used for batch norm and bias for fully connected layer
    std::vector<float> load1D(const std::string &key) const;

private:
    std::string filePath;
};



#endif //MODELLOADINGHELPER_H

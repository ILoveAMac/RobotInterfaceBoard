//
// Created by Wihan on 2024-09-08.
//

#include "ModelLoadingHelper.h"

ModelLoadingHelper::ModelLoadingHelper(const std::string &filePath) {
    this->filePath = filePath;
}

ModelLoadingHelper::~ModelLoadingHelper() {}

std::vector<std::vector<std::vector<std::vector<float> > > > ModelLoadingHelper::loadConv4D(
    const std::string &key) const {
    // Create file path by appending key to this->filepath
    const std::string fullPath = this->filePath + key;

    std::vector<std::vector<std::vector<std::vector<float> > > > convData;
    std::ifstream inFile(fullPath, std::ios::binary);
    if (!inFile) {
        std::cout << "Error opening file: " << fullPath << std::endl;
        throw std::runtime_error("Error opening file: " + fullPath);
    }

    cereal::BinaryInputArchive archive(inFile);
    archive(convData);

    std::cout << "Successfully deserialized 4D vector from: " << fullPath << std::endl;

    return convData;
}

std::vector<std::vector<float> > ModelLoadingHelper::loadFCL(const std::string &key) const {
    // Create file path by appending key to this->filepath
    const std::string fullPath = this->filePath + key;

    std::vector<std::vector<float> > fcData;
    std::ifstream inFile(fullPath, std::ios::binary);
    if (!inFile) {
        std::cout << "Error opening file: " << fullPath << std::endl;
        throw std::runtime_error("Error opening file: " + fullPath);
    }

    cereal::BinaryInputArchive archive(inFile);
    archive(fcData);

    std::cout << "Successfully deserialized 2D vector from: " << fullPath << std::endl;

    return fcData;
}

std::vector<float> ModelLoadingHelper::load1D(const std::string &key) const {
    // Create file path by appending key to this->filepath
    const std::string fullPath = this->filePath + key;

    std::vector<float> data;
    std::ifstream inFile(fullPath, std::ios::binary);
    if (!inFile) {
        std::cout << "Error opening file: " << fullPath << std::endl;
        throw std::runtime_error("Error opening file: " + fullPath);
    }

    cereal::BinaryInputArchive archive(inFile);
    archive(data);

    std::cout << "Successfully deserialized 1D vector from: " << fullPath << std::endl;

    return data;
}

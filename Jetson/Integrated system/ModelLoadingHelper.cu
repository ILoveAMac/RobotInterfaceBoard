//
// Created by Wihan on 2024-09-08.
//

#include "ModelLoadingHelper.cuh"

ModelLoadingHelper::ModelLoadingHelper(const std::string &filePath)
{
    this->filePath = filePath;
}

ModelLoadingHelper::~ModelLoadingHelper() {}

// Method to load 4D convolution weights and convert to __half
std::vector<std::vector<std::vector<std::vector<__half>>>> ModelLoadingHelper::loadConv4D(const std::string &key) const
{
    // Create file path by appending key to this->filepath
    const std::string fullPath = this->filePath + key;

    // Temporary variable to load float data
    std::vector<std::vector<std::vector<std::vector<float>>>> convData;
    std::ifstream inFile(fullPath, std::ios::binary);
    if (!inFile)
    {
        std::cerr << "Error opening file: " << fullPath << std::endl;
        throw std::runtime_error("Error opening file: " + fullPath);
    }

    // Deserialize the data using Cereal
    cereal::BinaryInputArchive archive(inFile);
    archive(convData);

    std::cout << "Successfully deserialized 4D vector from: " << fullPath << std::endl;

    // Initialize the 4D __half vector with the same dimensions
    std::vector<std::vector<std::vector<std::vector<__half>>>> convData_half;
    convData_half.reserve(convData.size());

    for (const auto &conv_channel : convData)
    {
        std::vector<std::vector<std::vector<__half>>> conv_channel_half;
        conv_channel_half.reserve(conv_channel.size());

        for (const auto &conv_filter : conv_channel)
        {
            std::vector<std::vector<__half>> conv_filter_half;
            conv_filter_half.reserve(conv_filter.size());

            for (const auto &conv_row : conv_filter)
            {
                std::vector<__half> conv_row_half;
                conv_row_half.reserve(conv_row.size());

                for (const auto &value : conv_row)
                {
                    conv_row_half.emplace_back(__float2half(value));
                }

                conv_filter_half.emplace_back(std::move(conv_row_half));
            }

            conv_channel_half.emplace_back(std::move(conv_filter_half));
        }

        convData_half.emplace_back(std::move(conv_channel_half));
    }

    return convData_half;
}

// Method to load 2D fully connected layer data and convert to __half
std::vector<std::vector<__half>> ModelLoadingHelper::loadFCL(const std::string &key) const
{
    // Create file path by appending key to this->filepath
    const std::string fullPath = this->filePath + key;

    // Temporary variable to load float data
    std::vector<std::vector<float>> fcData;
    std::ifstream inFile(fullPath, std::ios::binary);
    if (!inFile)
    {
        std::cerr << "Error opening file: " << fullPath << std::endl;
        throw std::runtime_error("Error opening file: " + fullPath);
    }

    // Deserialize the data using Cereal
    cereal::BinaryInputArchive archive(inFile);
    archive(fcData);

    std::cout << "Successfully deserialized 2D vector from: " << fullPath << std::endl;

    // Initialize the 2D __half vector with the same dimensions
    std::vector<std::vector<__half>> fcData_half;
    fcData_half.reserve(fcData.size());

    for (const auto &fc_row : fcData)
    {
        std::vector<__half> fc_row_half;
        fc_row_half.reserve(fc_row.size());

        for (const auto &value : fc_row)
        {
            fc_row_half.emplace_back(__float2half(value));
        }

        fcData_half.emplace_back(std::move(fc_row_half));
    }

    return fcData_half;
}

// Method to load 1D data and convert to __half
std::vector<__half> ModelLoadingHelper::load1D(const std::string &key) const
{
    // Create file path by appending key to this->filepath
    const std::string fullPath = this->filePath + key;

    // Temporary variable to load float data
    std::vector<float> data;
    std::ifstream inFile(fullPath, std::ios::binary);
    if (!inFile)
    {
        std::cerr << "Error opening file: " << fullPath << std::endl;
        throw std::runtime_error("Error opening file: " + fullPath);
    }

    // Deserialize the data using Cereal
    cereal::BinaryInputArchive archive(inFile);
    archive(data);

    std::cout << "Successfully deserialized 1D vector from: " << fullPath << std::endl;

    // Initialize the 1D __half vector with the same dimensions
    std::vector<__half> data_half;
    data_half.reserve(data.size());

    for (const auto &value : data)
    {
        data_half.emplace_back(__float2half(value));
    }

    return data_half;
}

//
// Created by Wihan on 2024-09-25.
//

#include "RDP.cuh"

#include <algorithm>

RDP::RDP()
{
}

RDP::~RDP()
{
}

std::vector<std::tuple<int, int>> RDP::ramerDouglasPeucker(const std::vector<std::tuple<int, int>> &contour,
                                                           const float epsilon)
{
    // Find the point with the maximum distance
    std::tuple<int, int> startPoint = contour[0];
    std::tuple<int, int> endPoint = contour[contour.size() - 1];

    float maxDistance = 0;
    int index = -1;

    for (int i = 1; i < contour.size() - 1; i++)
    {
        auto point = contour[i];

        auto distance = perpendicularDistance(point, startPoint, endPoint);

        if (distance > maxDistance)
        {
            index = i;
            maxDistance = distance;
        }
    }

    // If the maximum distance is greater than epsilon, recursively simplify
    std::vector<std::tuple<int, int>> result;
    if (maxDistance > epsilon)
    {
        // Recursive call for the two segments
        const std::vector firstHalf(contour.begin(), contour.begin() + index + 1);
        const std::vector secondHalf(contour.begin() + index, contour.end());

        // Recursively apply RDP on both halves
        auto result1 = ramerDouglasPeucker(firstHalf, epsilon);
        auto result2 = ramerDouglasPeucker(secondHalf, epsilon);

        // Append result1 to result (except the last point to avoid duplication)
        result.insert(result.end(), result1.begin(), result1.end() - 1);

        // Append result2 to result
        result.insert(result.end(), result2.begin(), result2.end());
    }
    else
    {
        // If the distance is below the threshold, return the endpoints only
        result.push_back(startPoint);
        result.push_back(endPoint);
    }

    return result;
}

float RDP::arcLength(const std::vector<std::tuple<int, int>> &contour)
{
    const auto startPoint = contour[0];

    // Calculate the euclideanDistance between each point in the contour and add them together
    std::tuple<int, int> prevPoint = startPoint;
    float distance = 0;
    for (auto point : contour)
    {
        if (point == prevPoint)
        {
            continue;
        }

        // Calculate distance between the two points
        distance += euclideanDistance(prevPoint, point);
        prevPoint = point;
    }

    // Add the distance between the prevPoint and the start point as the contour is closed
    distance += euclideanDistance(prevPoint, startPoint);

    return distance;
}

float RDP::perpendicularDistance(const std::tuple<int, int> &point, const std::tuple<int, int> &start,
                                 const std::tuple<int, int> &end)
{
    // Extract point coordinates
    const int x = std::get<0>(point);
    const int y = std::get<1>(point);

    const int x1 = std::get<0>(start);
    const int y1 = std::get<1>(start);

    const int x2 = std::get<0>(end);
    const int y2 = std::get<1>(end);

    // Calculate the area of the triangle formed by the line and the point
    // and divide by the length of the base (line segment)

    if (x1 == x2 && y1 == y2)
    {
        return sqrt(pow((x - x1), 2) + pow((y - y1), 2));
    }

    const float numerator = std::fabs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1);
    const float denominator = std::sqrt(static_cast<float>((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1)));

    return denominator == 0 ? 0 : numerator / denominator; // Avoid div by 0
}

float RDP::euclideanDistance(const std::tuple<int, int> &start, const std::tuple<int, int> &end)
{
    const int x1 = std::get<0>(start);
    const int y1 = std::get<1>(start);

    const int x2 = std::get<0>(end);
    const int y2 = std::get<1>(end);

    return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

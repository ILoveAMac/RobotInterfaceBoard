//
// Created by Wihan on 2024-09-25.
//

#include "markerIsolator.cuh"

markerIsolator::markerIsolator()
{
}

markerIsolator::~markerIsolator()
{
}

std::vector<std::vector<std::vector<int>>> markerIsolator::isolateMarkersFromContours(
    const std::vector<std::vector<int>> &contours)
{
    std::vector<std::vector<std::tuple<int, int>>> quads;

    // Step 1: Simplify all contours into quads
    for (const auto &contour : contours)
    {
        auto polygon = getSimplifiedPolygonFromContour(contour);
        if (isQuad(polygon))
        {
            quads.push_back(polygon);
        }
    }

    // Step 2: Find and store nested quad pairs
    std::vector<std::vector<std::vector<int>>> markers;

    for (size_t i = 0; i < quads.size(); ++i)
    {
        for (size_t j = 0; j < quads.size(); ++j)
        {
            if (i != j)
            {
                // Check if quad j is inside quad i
                if (isQuadAInsideQuadB(quads[j], quads[i]))
                {
                    // Convert the nested quads into integer coordinates and store them
                    std::vector<std::vector<int>> nestedPair;

                    // Parent quad
                    std::vector<int> parentQuad;
                    for (const auto &point : quads[i])
                    {
                        parentQuad.push_back(std::get<0>(point)); // x-coordinate
                        parentQuad.push_back(std::get<1>(point)); // y-coordinate
                    }
                    nestedPair.push_back(parentQuad);

                    // Child quad
                    std::vector<int> childQuad;
                    for (const auto &point : quads[j])
                    {
                        childQuad.push_back(std::get<0>(point)); // x-coordinate
                        childQuad.push_back(std::get<1>(point)); // y-coordinate
                    }
                    nestedPair.push_back(childQuad);

                    markers.push_back(nestedPair);
                }
            }
        }
    }

    // return markers;
    return removeInvalidMarkers(markers);
}

std::vector<std::tuple<int, int>> markerIsolator::getSimplifiedPolygonFromContour(
    std::vector<int> contour)
{
    RDP rdp;

    std::vector<std::tuple<int, int>> contourPoints;
    for (size_t i = 0; i < contour.size(); i += 2)
    {
        contourPoints.emplace_back(contour[i], contour[i + 1]);
    }
    contourPoints = ensureClosedContour(contourPoints);

    const float epsilon = 0.05 * rdp.arcLength(contourPoints);
    // Adjust this value to control the level of simplification
    std::vector<std::tuple<int, int>> simplifiedPolygon = rdp.ramerDouglasPeucker(contourPoints, epsilon);
    simplifiedPolygon.pop_back();

    return simplifiedPolygon;
}

bool markerIsolator::isQuad(const std::vector<std::tuple<int, int>> &quad)
{
    return quad.size() == 4;
}

float markerIsolator::calculateArea(const std::vector<std::tuple<int, int>> &quad)
{
    if (quad.size() != 4)
    {
        return 0;
    }

    const float length = euclideanDistance(quad[0], quad[1]);
    const float width = euclideanDistance(quad[1], quad[2]);

    return length * width;
}

std::vector<std::tuple<int, int>> markerIsolator::
    ensureClosedContour(const std::vector<std::tuple<int, int>> &contour)
{
    if (contour.size() < 2)
    {
        return contour; // A contour with fewer than 2 points cannot be closed
    }

    // Check if the start and end points are the same
    if (contour.front() == contour.back())
    {
        return contour; // Already closed
    }

    // Otherwise, close the contour by appending the first point to the end
    std::vector<std::tuple<int, int>> closedContour = contour;
    closedContour.push_back(contour.front());
    return closedContour;
}

float markerIsolator::euclideanDistance(const std::tuple<int, int> &start, const std::tuple<int, int> &end)
{
    const int x1 = std::get<0>(start);
    const int y1 = std::get<1>(start);

    const int x2 = std::get<0>(end);
    const int y2 = std::get<1>(end);

    return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

bool markerIsolator::isQuadAInsideQuadB(const std::vector<std::tuple<int, int>> &quadA,
                                        const std::vector<std::tuple<int, int>> &quadB)
{
    // QuadA must have an area less than quadB
    if (calculateArea(quadA) >= calculateArea(quadB))
    {
        return false;
    }

    // Check if all points of Quad A are inside Quad B
    for (auto pointA : quadA)
    {
        if (!isPointInsidePolygon(pointA, quadB))
        {
            return false;
        }
    }

    return true; // All points of Quad A are inside Quad B
}

bool markerIsolator::isPointInsidePolygon(const std::tuple<int, int> &point,
                                          const std::vector<std::tuple<int, int>> &polygon)
{
    const int x = std::get<0>(point);
    const int y = std::get<1>(point);

    bool inside = false;
    const int n = polygon.size();

    for (int i = 0, j = n - 1; i < n; j = i++)
    {
        const int xi = std::get<0>(polygon[i]);
        const int yi = std::get<1>(polygon[i]);
        const int xj = std::get<0>(polygon[j]);
        const int yj = std::get<1>(polygon[j]);

        // Check if point is inside by using the ray-casting algorithm
        const bool intersect = ((yi > y) != (yj > y)) &&
                               (x < (xj - xi) * (y - yi) / static_cast<float>(yj - yi) + xi);
        if (intersect)
        {
            inside = !inside;
        }
    }

    return inside;
}

std::vector<std::vector<std::vector<int>>> markerIsolator::removeInvalidMarkers(
    const std::vector<std::vector<std::vector<int>>> &potentialMarkers)
{
    // Define a threshold for how close the centroids should be
    constexpr float centroidDistanceThreshold = 10.0f; // Adjust this threshold as needed
    // To Define a threshold for how similar the aspect ratios should be
    constexpr float aspectRatioSimilarityThreshold = 0.05f; // Allowable percentage difference

    constexpr float minArea = 1000.0;

    std::vector<std::vector<std::vector<int>>> validMarkers;

    // Iterate through each pair of potential markers
    for (const auto &marker : potentialMarkers)
    {
        if (marker.size() != 2)
        {
            continue; // Skip if it doesn't consist of exactly two quads
        }

        // Convert the quads into tuples
        std::vector<std::tuple<int, int>> quadA, quadB;
        for (size_t i = 0; i < marker[0].size(); i += 2)
        {
            quadA.emplace_back(marker[0][i], marker[0][i + 1]);
        }
        for (size_t i = 0; i < marker[1].size(); i += 2)
        {
            quadB.emplace_back(marker[1][i], marker[1][i + 1]);
        }

        // Check that the area of Quad1 is large enough
        auto areaQ1 = calculateArea(quadA);
        printf("areaQ1: %f\n", areaQ1);
        if (areaQ1 < minArea)
        {
            continue;
        }

        // 1. Check if the centroids of the quad pair are close together
        auto centroidA = getQuadCentroid(quadA);
        auto centroidB = getQuadCentroid(quadB);

        const float distance = euclideanDistance(centroidA, centroidB);
        if (distance > centroidDistanceThreshold)
        {
            continue; // Skip this pair since the centroids are too far apart
        }

        // 2. Check if the aspect ratios of the quad pairs are similar
        float aspectRatioA = getQuadAspectRatio(quadA);
        float aspectRatioB = getQuadAspectRatio(quadB);

        // Calculate the absolute difference ratio
        float ratioDifference = fabs(aspectRatioA - aspectRatioB) / std::min(aspectRatioA, aspectRatioB);
        if (ratioDifference > aspectRatioSimilarityThreshold)
        {
            continue; // Skip this pair since the aspect ratios differ significantly
        }

        // If both checks are passed, this is a valid marker pair
        validMarkers.push_back(marker);
    }

    return validMarkers;
}

std::tuple<int, int> markerIsolator::getQuadCentroid(const std::vector<std::tuple<int, int>> &quad)
{
    int sumX = 0;
    int sumY = 0;

    // Iterate over the quad's vertices
    for (const auto &point : quad)
    {
        sumX += std::get<0>(point);
        sumY += std::get<1>(point);
    }

    // Calculate the average (centroid)
    int centroidX = sumX / quad.size();
    int centroidY = sumY / quad.size();

    // Return the centroid as a tuple
    return std::make_tuple(centroidX, centroidY);
}

float markerIsolator::getQuadAspectRatio(const std::vector<std::tuple<int, int>> &quad)
{
    if (quad.size() != 4)
    {
        // Invalid, not a quad
        return -1.0f;
    }

    // Extract the points of the quad
    const auto p1 = quad[0];
    const auto p2 = quad[1];
    const auto p3 = quad[2];
    const auto p4 = quad[3];

    // Calculate the lengths of the opposite sides
    const float width1 = std::hypot(std::get<0>(p2) - std::get<0>(p1), std::get<1>(p2) - std::get<1>(p1));
    const float width2 = std::hypot(std::get<0>(p4) - std::get<0>(p3), std::get<1>(p4) - std::get<1>(p3));

    const float height1 = std::hypot(std::get<0>(p3) - std::get<0>(p2), std::get<1>(p3) - std::get<1>(p2));
    const float height2 = std::hypot(std::get<0>(p4) - std::get<0>(p1), std::get<1>(p4) - std::get<1>(p1));

    // Calculate the average width and height
    const float avgWidth = (width1 + width2) / 2.0f;
    const float avgHeight = (height1 + height2) / 2.0f;

    // Calculate the aspect ratio
    const float aspectRatio = avgWidth / avgHeight;

    return aspectRatio;
}

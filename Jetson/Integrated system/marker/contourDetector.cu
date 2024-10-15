//
// Created by Wihan on 2024-09-24.
//

#include "contourDetector.cuh"

contourDetector::contourDetector() {
}

contourDetector::~contourDetector() {
}

std::vector<std::vector<int> > contourDetector::detect(const float *image, const int width, const int height) {
    // Vector to store detected contours
    std::vector<std::vector<int> > contours;

    // Create a 2D vector to track visited pixels
    std::vector visited(height, std::vector(width, false));

    // Define the directions for 8-connectivity (clockwise starting from left)
    const std::vector<std::pair<int, int> > directions = {
        {-1, 0}, {-1, 1}, {0, 1}, {1, 1},
        {1, 0}, {1, -1}, {0, -1}, {-1, -1}
    };

    // Helper function to find the next pixel in the contour
    auto findNextPixel = [&](int cx, int cy, int start_dir) {
        for (int i = 0; i < 8; ++i) {
            int ndir = (start_dir + i) % 8;
            int nx = cx + directions[ndir].first;
            int ny = cy + directions[ndir].second;

            // Check if this is a valid, unvisited edge pixel
            if (isValid(nx, ny, width, height) && image[ny * width + nx] > 0 && !visited[ny][nx]) {
                return std::make_tuple(nx, ny, ndir);
            }
        }
        return std::make_tuple(-1, -1, -1); // No valid next pixel found
    };

    // Start scanning the binary image to find contours
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Check for an unvisited edge pixel
            if (image[y * width + x] > 0 && !visited[y][x]) {
                std::vector<int> contour; // Start a new contour

                // Mark the starting point as visited
                visited[y][x] = true;
                contour.push_back(x);
                contour.push_back(y);

                // Start following the contour from the top-left neighbor (direction 7)
                int cx = x, cy = y;
                int current_dir = 7;

                std::vector<int> tempContour;
                while (true) {
                    // Find the next contour pixel
                    int nx, ny, new_dir;
                    std::tie(nx, ny, new_dir) = findNextPixel(cx, cy, current_dir);

                    if (nx == -1 || ny == -1) {
                        // Check if tempContour is not empty
                        if (!tempContour.empty() && tempContour.size() >= 4) {
                            const float startX = tempContour[0];
                            const float startY = tempContour[1];
                            const float endX = tempContour[tempContour.size() - 2];
                            const float endY = tempContour[tempContour.size() - 1];

                            const float distance = calculateDistance(startX, startY, endX, endY);

                            if (distance <= DISTANCE_THRESHOLD) {
                                // Points are close enough, add tempContour to contour
                                contour.insert(contour.end(), tempContour.begin(), tempContour.end());
                                tempContour.clear();
                            }
                        }
                        break;
                    }

                    // Mark the new pixel as visited
                    visited[ny][nx] = true;
                    tempContour.push_back(nx);
                    tempContour.push_back(ny);

                    // Update the current position and direction
                    cx = nx;
                    cy = ny;
                    current_dir = (new_dir + 6) % 8; // Adjust the direction

                    // If we are back to the starting point, stop
                    if (cx == x && cy == y && tempContour.size() >= 4) {
                        // Move the temp contour into the contour if it contains more than 4 points
                        contour.insert(contour.end(), tempContour.begin(), tempContour.end());
                        tempContour.clear();
                        break;
                    }
                }
                // Store the detected contour
                contours.push_back(contour);
            }
        }
    }
    return contours;
}

bool contourDetector::isValid(const int x, const int y, const int width, const int height) {
    return x >= 0 && x < width && y >= 0 && y < height;
}

float contourDetector::calculateDistance(const float x1, const float y1, const float x2, const float y2) {
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

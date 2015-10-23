//
// Created by vikram on 10/21/15.
//

#ifndef GFLIP_GFLIP_TYPES_HPP
#define GFLIP_GFLIP_TYPES_HPP

#include <geometry/point.h>
#include <vector>

typedef struct LaserScanInfo {
    double max_range;
    std::vector<double> rem_values;
    std::vector<double> angle;
    std::vector<double> distance;
    OrientedPoint2D robot_pose;
    OrientedPoint2D laser_pose;
} LaserScanInfo;

typedef struct WordResult {
    unsigned int word;
    OrientedPoint2D pose;
} WordResult;

#endif //GFLIP_GFLIP_TYPES_HPP

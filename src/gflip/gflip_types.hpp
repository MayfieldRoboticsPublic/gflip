//
// Created by vikram on 10/21/15.
//

#ifndef GFLIP_GFLIP_TYPES_HPP
#define GFLIP_GFLIP_TYPES_HPP

typedef struct LaserScanInfo {
    double max_range;
    std::vector<double> rem_values;
    std::vector<double> angle;
    std::vector<double> distance;
} LaserScanInfo;

typedef struct WordResult {
    unsigned int word;
    OrientedPoint2D pose;
} WordResult;

#endif //GFLIP_GFLIP_TYPES_HPP

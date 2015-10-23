//
// Created by vikram on 10/20/15.
//

#ifndef GFLIP_GFLIP_BOW_GENERATOR_HPP
#define GFLIP_GFLIP_BOW_GENERATOR_HPP
#include <feature/Detector.h>

#include <sensorstream/LogSensorStream.h>

#include <utils/SimpleMinMaxPeakFinder.h>
#include <utils/HistogramDistances.h>
#include <vocabulary/Vocabulary.h>

#include <boost/archive/binary_iarchive.hpp>

#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
#include <utility>
#include <map>

#include <sys/time.h>

#include "gflip_types.hpp"
namespace bow_generator {

    class gflip_bow_generator {
    public:
        gflip_bow_generator();

        void set_laser_pose(OrientedPoint2D pose);
        std::multimap<double, WordResult> get_signature();
        void generate_bow(std::vector<InterestPoint *> m_pointsReference, std::string vocabulary_file);

    private:
        HistogramVocabulary histogramVocabulary;

        OrientedPoint2D laser_pose; //Pose referenced
        std::multimap<double, WordResult> signature;
    };
}

#endif //GFLIP_GFLIP_BOW_GENERATOR_HPP

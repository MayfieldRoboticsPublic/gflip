//
// Created by vikram on 10/20/15.
//

#ifndef GFLIP_GFLIP_FEATURE_EXTRACTOR_HPP
#define GFLIP_GFLIP_FEATURE_EXTRACTOR_HPP


#include <feature/Detector.h>
#include <feature/RangeDetector.h>
#include <feature/CurvatureDetector.h>
#include <feature/NormalBlobDetector.h>
#include <feature/NormalEdgeDetector.h>
#include <sensorstream/CarmenLog.h>
#include <sensorstream/LogSensorStream.h>
#include <sensorstream/SensorStream.h>
#include <utils/SimpleMinMaxPeakFinder.h>
#include <utils/HistogramDistances.h>

#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
#include <utility>

#include <sys/time.h>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <boost/multi_array.hpp>
#include "gflip_types.hpp"
namespace feature_extractor {


    class gflip_feature_extractor {
    public:
        gflip_feature_extractor();

        void set_robot_pose(double x, double y, double theta);

        void set_laser_pose(double x, double y, double theta);

        void set_scan(LaserScanInfo scan);

        OrientedPoint2D get_laser_pose();

        std::vector<InterestPoint *> get_interest_points();

        void extract_features(int detectorType, int descriptorType, int distanceType);

    private:
        OrientedPoint2D robot_pose;
        OrientedPoint2D laser_pose; //Pose referenced

        CurvatureDetector *m_detectorCurvature = NULL;
        NormalBlobDetector *m_detectorNormalBlob = NULL;
        NormalEdgeDetector *m_detectorNormalEdge = NULL;
        RangeDetector *m_detectorRange = NULL;
        Detector *m_detector = NULL;

        DescriptorGenerator *m_descriptor = NULL;

        std::vector<InterestPoint *> m_pointReference;

        LaserReading *current_scan = NULL;
    };
}

#endif //GFLIP_GFLIP_FEATURE_EXTRACTOR_HPP

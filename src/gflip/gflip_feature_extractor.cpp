//
// Created by vikram on 10/20/15.
//

#include "gflip_feature_extractor.hpp"
#include <feature/ShapeContext.h>
#include <feature/BetaGrid.h>
using namespace feature_extractor;

gflip_feature_extractor::gflip_feature_extractor()
{}

void gflip_feature_extractor::set_scan(LaserScanInfo scan)
{
  if(current_scan == NULL) {
      current_scan = new LaserReading(scan.angle, scan.distance);
  }
  else {
    current_scan->setRho(scan.distance);
    current_scan->setPhi(scan.angle);
  }
  current_scan->setLaserPose(scan.laser_pose);
  current_scan->setRobotPose(scan.robot_pose);
  current_scan->setMaxRange(scan.max_range);
  current_scan->setRemission(scan.rem_values);

}

OrientedPoint2D gflip_feature_extractor::get_laser_pose()
{
    return laser_pose;
}

std::vector<InterestPoint *> gflip_feature_extractor::get_interest_points()
{
    return  m_pointReference;
}

void gflip_feature_extractor::extract_features(int detectorType, int descriptorType, int distanceType) {

    unsigned int scale = 5, dmst = 2, window = 3;
    double baseSigma = 0.2, sigmaStep = 1.4, minPeak = 0.34, minPeakDistance = 0.001;
    bool useMaxRange = false, gspan = false;
    SimpleMinMaxPeakFinder *m_peakMinMax = new SimpleMinMaxPeakFinder(minPeak, minPeakDistance);

    BetaGridGenerator *m_betaGenerator = NULL;
    ShapeContextGenerator *m_shapeGenerator = NULL;
    std::string detector("");
    switch(detectorType){
        case 0:
            m_detectorCurvature = new CurvatureDetector(m_peakMinMax, scale, baseSigma, sigmaStep, dmst);
            m_detectorCurvature->setUseMaxRange(useMaxRange);
            m_detector = m_detectorCurvature;
            detector = "curvature";
            break;
        case 1:
            m_detectorNormalEdge = new NormalEdgeDetector(m_peakMinMax, scale, baseSigma, sigmaStep, window);
            m_detector = m_detectorNormalEdge;
            detector = "edge";
            break;
        case 2:
            m_detectorNormalBlob = new NormalBlobDetector(m_peakMinMax, scale, baseSigma, sigmaStep, window);
            m_detector = m_detectorNormalBlob;
            detector = "blob";
            break;
        case 3:
            m_detectorRange = new RangeDetector(m_peakMinMax, scale, baseSigma, sigmaStep);
            m_detector = m_detectorRange;
            detector = "range";
            break;
        default:
            std::cerr << "Wrong detector type" << std::endl;
            exit(-1);
    }

    HistogramDistance<double> *dist = NULL;

    std::string distance("");
    switch(distanceType){
        case 0:
            dist = new EuclideanDistance<double>();
            distance = "euclid";
            break;
        case 1:
            dist = new Chi2Distance<double>();
            distance = "chi2";
            break;
        case 2:
            dist = new SymmetricChi2Distance<double>();
            distance = "symchi2";
            break;
        case 3:
            dist = new BatthacharyyaDistance<double>();
            distance = "batt";
            break;
        case 4:
            dist = new KullbackLeiblerDistance<double>();
            distance = "kld";
            break;
        case 5:
            dist = new JensenShannonDistance<double>();
            distance = "jsd";
            break;
        default:
            std::cerr << "Wrong distance type" << std::endl;
            exit(-1);
    }

    std::string descriptor("");
    switch(descriptorType){
        case 0:
            m_betaGenerator = new BetaGridGenerator(0.02, 0.5, 4, 12);
            m_betaGenerator->setDistanceFunction(dist);
            m_descriptor = m_betaGenerator;
            descriptor = "beta";
            break;
        case 1:
            m_shapeGenerator = new ShapeContextGenerator(0.02, 0.5, 4, 12);
            m_shapeGenerator->setDistanceFunction(dist);
            m_descriptor = m_shapeGenerator;
            descriptor = "shape";
            break;
        default:
            std::cerr << "Wrong descriptor type" << std::endl;
            exit(-1);
    }

    m_pointReference.resize(current_scan->getPhi().size());

    if(current_scan){
        m_detector->detect(*current_scan, m_pointReference);
    }

    for(int desc_i = 0; desc_i < m_pointReference.size(); desc_i++)
    {
        m_pointReference[desc_i]->setDescriptor(m_descriptor->describe(*m_pointReference[desc_i], *current_scan));
    }

}

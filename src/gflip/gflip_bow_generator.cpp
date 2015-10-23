//
// Created by vikram on 10/20/15.
//

#include "gflip_bow_generator.hpp"


using namespace bow_generator;

gflip_bow_generator::gflip_bow_generator() { }

void gflip_bow_generator::set_laser_pose(OrientedPoint2D pose)
{
    laser_pose = pose;
}

std::multimap<double, WordResult> gflip_bow_generator::get_signature()
{
    return signature;
}

void gflip_bow_generator::generate_bow(std::vector<InterestPoint *> m_pointsReference, std::string vocabulary_file) {

    std::string vocabulary("Vocabulary.voc"); //TODO make this an arguement so we can pass our own vocab
    std::ifstream vocabularyStream(vocabulary.c_str());
    boost::archive::binary_iarchive vocabularyArchive(vocabularyStream);
    vocabularyArchive >> histogramVocabulary;

    for(unsigned int point_i = 0; point_i < m_pointsReference.size(); point_i++)
    {

        InterestPoint * point = m_pointsReference[point_i];
        OrientedPoint2D localpose = laser_pose.ominus(point->getPosition());
        double angle = atan2(localpose.y, localpose.x);
        unsigned int bestWord = 0;
        double bestMatch = 0.;
        std::vector<double> descriptor;
        std::vector<double> weights;
        point->getDescriptor()->getWeightedFlatDescription(descriptor, weights);
        HistogramFeatureWord word(descriptor, NULL, weights);
        for(unsigned int w = 0; w < histogramVocabulary.size(); w++) {
            double score = histogramVocabulary[w].sim(&word);
            if(score > bestMatch) {
                bestMatch = score;
                bestWord = w;
            }
        }
        WordResult best; best.pose = localpose; best.word = bestWord;
        signature.insert(std::make_pair(angle,best));

    }


}
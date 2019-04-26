#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/collisionAlgorithm/data/DataDistanceMeasure.h>
#include <sofa/collisionAlgorithm/data/DataDetectionOutput.h>

namespace sofa {

namespace collisionAlgorithm {

/*!
 * \brief The ContactDistance class
 * Applies specified algorithm on 'from' and 'dest' geometry
 */
class ContactDistance : public sofa::core::objectmodel::BaseObject {
public:
    SOFA_CLASS(ContactDistance , sofa::core::objectmodel::BaseObject);

    Data<DistanceMeasure> d_distance;

    /*!
     * \brief ContactDistance constructor
     */
    ContactDistance()
        : d_distance(initData(&d_distance, DistanceMeasure(std::bind(&ContactDistance::computeDistance, this, std::placeholders::_1)), "distance", "Link to detection output")){}

    /*!
     * \brief The ContactNormal class is the container class for direction constraints
     */
    double computeDistance(const collisionAlgorithm::PairDetection & d) {
        defaulttype::Vector3 PQ = d.second->getPosition()-d.first->getPosition();
        defaulttype::Vector3 N = d.first->getNormal();
        return dot(PQ,N);
    }

};

}

}

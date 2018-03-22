#pragma once

#include <algorithm/CollisionDetectionAlgorithm.h>
#include <limits>

namespace collisionAlgorithm {

CollisionDetectionAlgorithm::CollisionDetectionAlgorithm()
: p_from(this)
, p_dest(this)
{}

PairProximity  CollisionDetectionAlgorithm::getClosestPoint(ConstraintElementPtr efrom) {
    double min_dist = std::numeric_limits<double>::max();

    ConstraintProximityPtr pfrom = efrom->getControlPoint(0);
    Vector3 P = pfrom->getPosition();
    PairProximity min_pair;

    for (unsigned i=0;i<p_dest->getNbElements();i++) {
        ConstraintElementPtr edest = p_dest->getElement(i);
        ConstraintProximityPtr pdest = edest->project(P);
        pfrom = efrom->project(pdest->getPosition());

        //iterate until to find the correct location on pfrom
        for (int itearation = 0;itearation<10 && (P-pfrom->getPosition()).norm()>0.0001;itearation++) {
            P = pfrom->getPosition();
            pdest = edest->project(P);
            pfrom = efrom->project(pdest->getPosition());
        }

        //compute all the distances with to elements
        double dist = (pfrom->getPosition() - pdest->getPosition()).norm();

        if (dist<min_dist) {
            min_dist = dist;
            min_pair.first = pfrom;
            min_pair.second = pdest;
        }
    }

    return min_pair;
}

void CollisionDetectionAlgorithm::processAlgorithm() {
    for (unsigned i=0;i<p_from->getNbElements();i++) {
        PairProximity pair = getClosestPoint(p_from->getElement(i));

        if (pair.first == NULL) continue;
        if (pair.second == NULL) continue;

        m_pairDetection.push_back(pair);
    }
}

}

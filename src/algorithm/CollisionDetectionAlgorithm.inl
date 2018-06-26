#pragma once

#include <algorithm/CollisionDetectionAlgorithm.h>
#include <limits>
#include <geometry/AABBGeometry.h>

namespace collisionAlgorithm {

CollisionDetectionAlgorithm::CollisionDetectionAlgorithm()
: p_from("from",RIGHT,this)
, p_dest("dest",RIGHT,this)
{}

PairProximity  CollisionDetectionAlgorithm::getClosestPoint(ConstraintElementPtr efrom) {
    PairProximity min_pair;
    double min_dist = std::numeric_limits<double>::max();

    ConstraintProximityPtr pfrom = efrom->getControlPoint();
    if (pfrom == NULL) return min_pair;
    Vector3 P = pfrom->getPosition();

    for (unsigned i=0;i<p_dest->getNbElements();i++) {
        ConstraintElementPtr edest = p_dest->getElement(i);
        ConstraintProximityPtr pdest = edest->project(P);
        pfrom = efrom->project(pdest->getPosition());

        //iterate until to find the correct location on pfrom
        for (int itearation = 0;itearation<10 && pfrom->distance(P)>0.0001;itearation++) {
            P = pfrom->getPosition();
            pdest = edest->project(P);
            pfrom = efrom->project(pdest->getPosition());
        }

        //compute all the distances with to elements
        double dist = pdest->distance(P);

        if (dist<min_dist) {
            min_dist = dist;
            min_pair.first = pfrom;
            min_pair.second = pdest;
        }
    }

    return min_pair;
}


void CollisionDetectionAlgorithm::processAlgorithm() {
    m_pairDetection.clear();

    if (AABBGeometry * geo = dynamic_cast<AABBGeometry*>(p_dest())) {
        for (unsigned i=0;i<p_from->getNbElements();i++) {
            ConstraintElementPtr efrom = p_from->getElement(i);
            ConstraintProximityPtr pfrom = efrom->getControlPoint();
            if (pfrom == NULL) continue;
            ConstraintProximityPtr pdest = geo->project(pfrom->getPosition());
            if (pdest == NULL) continue;
            m_pairDetection.push_back(PairProximity(pfrom,pdest));
        }
    } else {
        for (unsigned i=0;i<p_from->getNbElements();i++) {
            PairProximity pair = getClosestPoint(p_from->getElement(i));

            if (pair.first == NULL) continue;
            if (pair.second == NULL) continue;

            m_pairDetection.push_back(pair);
        }
    }
}

}

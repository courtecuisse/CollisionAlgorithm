#pragma once

#include <sofa/collisionAlgorithm/algorithm/CollisionDetectionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseFilter.h>
#include <sofa/collisionAlgorithm/iterators/SubsetElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

void CollisionDetectionAlgorithm::computeCollisionReset() {
    d_output.beginEdit()->clear();
    d_output.endEdit();
}

BaseElementIterator::UPtr CollisionDetectionAlgorithm::getDestIterator(const defaulttype::Vector3 & P) {
    const BroadPhase * decorator = d_dest.getValue().getBroadPhase();

    if (decorator == NULL) return d_dest.getValue().begin();
    else {
        defaulttype::BoundingBox bbox = decorator->getBBox();

        //project P inside the bbox
        defaulttype::Vector3 cbox;
        cbox[0] = std::min(std::max(P[0],bbox.minBBox()[0]),bbox.maxBBox()[0]);
        cbox[1] = std::min(std::max(P[1],bbox.minBBox()[1]),bbox.maxBBox()[1]);
        cbox[2] = std::min(std::max(P[2],bbox.minBBox()[2]),bbox.maxBBox()[2]);

        unsigned d = 0;
        std::set<unsigned> selectedElements;
        while (selectedElements.empty() && decorator->selectElement(P,selectedElements,d)) {
            d++;// we look for boxed located at d+1
        }

        if (selectedElements.empty()) return d_dest.getValue().begin();

        return BaseElementIterator::UPtr(new SubsetElementIterator(d_dest.getValue().end(),selectedElements));
    }
}

DetectionOutput::PairDetection CollisionDetectionAlgorithm::findClosestPoint(const BaseElementIterator::UPtr & itfrom)
{
    double min_dist = std::numeric_limits<double>::max();
    DetectionOutput::PairDetection min_pair(nullptr,nullptr);

    defaulttype::Vector3 P = itfrom->center()->getPosition();

    BaseElementIterator::UPtr itdest=getDestIterator(P);

    while (itdest != d_dest.getValue().end())
    {
        BaseProximity::SPtr pdest = itdest->project(P);
        BaseProximity::SPtr pfrom  = itfrom->project(pdest->getPosition()); // reproject one on the initial proximity

//        //iterate until to find the correct location on pfrom
//        for (int itearation = 0;itearation<10 && pfrom->distance(P)>0.0001;itearation++) {
//            P = pfrom->getPosition();
//            pdest = edest->project(P);
//            pfrom = it_element.efrom->projectsofasc   (pdest->getPosition());
//        }

        if (acceptFilter(pfrom,pdest)) {
            defaulttype::Vector3 N = pfrom->getPosition() - pdest->getPosition();
            double dist = N.norm();

            if (dist<min_dist)
            {
                min_dist = dist;
                min_pair.first = pfrom;
                min_pair.second = pdest;
            }
        }

        itdest++;
    }

    return min_pair;
}

void CollisionDetectionAlgorithm::computeCollisionDetection()
{
    DetectionOutput * output = d_output.beginEdit();

    for (auto itfrom=d_from.getValue().begin();itfrom!=d_from.getValue().end();itfrom++) {
        DetectionOutput::PairDetection min_pair = findClosestPoint(itfrom);

        if (min_pair.first == nullptr || min_pair.second == nullptr) continue;

        output->add(min_pair.first,min_pair.second);
    }

    d_output.endEdit();
}

}

}

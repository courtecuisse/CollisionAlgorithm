#pragma once

#include <sofa/collisionAlgorithm/algorithm/FindClosestPointAlgorithm.h>
#include <sofa/collisionAlgorithm/iterators/SubsetElementIterator.h>
#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>

namespace sofa
{

namespace collisionAlgorithm
{

void FindClosestPointAlgorithm::computeCollisionReset() {
    d_output.beginEdit()->clear();
    d_output.endEdit();
}

BaseElementIterator::UPtr FindClosestPointAlgorithm::getDestIterator(const defaulttype::Vector3 & P) {
    const BroadPhase * decorator = l_dest->getBroadPhase();

    if (decorator == NULL) return l_dest->begin();
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

        if (selectedElements.empty()) return l_dest->begin();

        return BaseElementIterator::UPtr(new SubsetElementIterator(l_dest.get(),selectedElements));
    }
}

DetectionOutput::PairDetection FindClosestPointAlgorithm::findClosestPoint(const BaseElement::UPtr & elfrom)
{
    double min_dist = std::numeric_limits<double>::max();
    DetectionOutput::PairDetection min_pair(nullptr,nullptr);

    defaulttype::Vector3 P = elfrom->center()->getPosition();

    BaseElementIterator::UPtr itdest=getDestIterator(P);

    while (itdest != l_dest->end())
    {
        BaseProximity::SPtr pdest = (*itdest)->project(P);
        BaseProximity::SPtr pfrom  = elfrom->project(pdest->getPosition()); // reproject one on the initial proximity

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

void FindClosestPointAlgorithm::computeCollisionDetection()
{
    if (l_from == NULL) return;
    if (l_dest == NULL) return;

    DetectionOutput * output = d_output.beginEdit();

    for (auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++) {
        DetectionOutput::PairDetection min_pair = findClosestPoint(*itfrom);

        if (min_pair.first == nullptr || min_pair.second == nullptr) continue;

        output->add(min_pair.first,min_pair.second);
    }

    d_output.endEdit();
}

}

}

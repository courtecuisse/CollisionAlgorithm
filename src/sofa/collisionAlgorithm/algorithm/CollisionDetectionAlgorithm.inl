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

static BaseElementIterator::UPtr broadPhaseBegin(const defaulttype::Vector3 & P, const BaseGeometry * dest) {
    BroadPhase * decorator = dest->getBroadPhase();

    if (decorator == NULL) return dest->begin();
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

        if (selectedElements.empty()) return dest->begin();

        return BaseElementIterator::UPtr(new SubsetElementIterator(dest,selectedElements));
    }
}

DetectionOutput::PairDetection CollisionDetectionAlgorithm::findClosestPoint(const BaseElementIterator::UPtr & itfrom, const BaseGeometry * dest,const std::set<BaseFilter*> & filters)
{
    double min_dist = std::numeric_limits<double>::max();
    DetectionOutput::PairDetection min_pair(nullptr,nullptr);

    defaulttype::Vector3 P = itfrom->center()->getPosition();

    BaseElementIterator::UPtr itdest=broadPhaseBegin(P,dest);

    while (! itdest->end(dest))
    {
        BaseProximity::SPtr pdest = itdest->project(P);
        BaseProximity::SPtr pfrom  = itfrom->project(pdest->getPosition()); // reproject one on the initial proximity

//        //iterate until to find the correct location on pfrom
//        for (int itearation = 0;itearation<10 && pfrom->distance(P)>0.0001;itearation++) {
//            P = pfrom->getPosition();
//            pdest = edest->project(P);
//            pfrom = it_element.efrom->projectsofasc   (pdest->getPosition());
//        }

        bool accept_filter = true;
        for (auto itfilter=filters.cbegin();accept_filter && (itfilter != filters.cend());itfilter++) {
            accept_filter = (*itfilter)->accept(pdest,pfrom);
        }

        if (accept_filter) {
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

    for (auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++) {
        DetectionOutput::PairDetection min_pair = findClosestPoint(itfrom, l_dest.get(), getFilters());

        if (min_pair.first == nullptr || min_pair.second == nullptr) continue;

        output->add(min_pair.first,min_pair.second);
    }

    d_output.endEdit();
}

}

}

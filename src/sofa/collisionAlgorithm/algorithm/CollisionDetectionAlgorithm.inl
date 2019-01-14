#pragma once

#include <sofa/collisionAlgorithm/algorithm/CollisionDetectionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElementFilter.h>
#include <sofa/collisionAlgorithm/iterator/SubsetIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

void CollisionDetectionAlgorithm::computeCollisionReset() {
    d_output.beginEdit()->clear();
    d_output.endEdit();
}

DetectionOutput::PairDetection CollisionDetectionAlgorithm::findClosestPoint(BaseElement::Iterator & itfrom)
{
    double min_dist = std::numeric_limits<double>::max();
    DetectionOutput::PairDetection min_pair(nullptr,nullptr);

//    BaseProximity::SPtr pfrom = it_element->getFrom()->getControlPoint(); //centered control point
//    if (pfrom == nullptr)
//        return ;


    defaulttype::Vector3 P = itfrom->center()->getPosition();

    for (BaseElement::Iterator it = begin(P);it!=l_dest->end();it++)
    {
        BaseProximity::SPtr pdest = it->project(P);
        BaseProximity::SPtr pfrom  = itfrom->project(pdest->getPosition());

//        pfrom = it_element.efrom->project(pdest->getPosition());
//        //iterate until to find the correct location on pfrom
//        for (int itearation = 0;itearation<10 && pfrom->distance(P)>0.0001;itearation++) {
//            P = pfrom->getPosition();
//            pdest = edest->project(P);
//            pfrom = it_element.efrom->project(pdest->getPosition());
//        }

        //compute all the distances with to elements
        defaulttype::Vector3 N = pfrom->getPosition() - pdest->getPosition();
        double dist = N.norm();

        if (dist > d_minDist.getValue())
            continue;
//        if (dot(pfrom->getNormal(),pdest->getNormal()) > -d_minAngle.getValue())
//            continue;

        if (dist<min_dist)
        {
            min_dist = dist;
            min_pair.first = pfrom;
            min_pair.second = pdest;
        }
    }

    return min_pair;
}

BaseElement::Iterator CollisionDetectionAlgorithm::begin(const defaulttype::Vector3 & P) const {
    BaseDecorator * decorator = l_dest->getBroadPhase();

    if (decorator == NULL) return l_dest->begin();

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

    return BaseElement::Iterator(new SubsetIterator(l_dest.get(),selectedElements));
}


void CollisionDetectionAlgorithm::computeCollisionDetection()
{
    DetectionOutput * output = d_output.beginEdit();

    for (auto it=l_from->begin();it!=l_from->end();it++) {
        DetectionOutput::PairDetection min_pair = findClosestPoint(it);

        if (min_pair.first == nullptr || min_pair.second == nullptr) continue;

        output->add(min_pair.first,min_pair.second);
    }

    d_output.endEdit();
}

}

}

#pragma once

#include <sofa/collisionAlgorithm/algorithm/CollisionDetectionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElementFilter.h>
#include <sofa/collisionAlgorithm/iterator/SubsetIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

void CollisionDetectionAlgorithm::findClosestPoint(BaseProximity::SPtr pfrom, BaseElement::Iterator & it)
{
    std::pair<BaseProximity::SPtr,BaseProximity::SPtr> min_pair;
    double min_dist = std::numeric_limits<double>::max();
    min_pair.first = nullptr;
    min_pair.second = nullptr;

//    BaseProximity::SPtr pfrom = it_element->getFrom()->getControlPoint(); //centered control point
//    if (pfrom == nullptr)
//        return ;


    defaulttype::Vector3 P = pfrom->getPosition();

    for (;it!=l_dest->end();it++)
    {
        BaseProximity::SPtr pdest = it->project(P);

//        pfrom = it_element.efrom->project(pdest->getPosition());
//        //iterate until to find the correct location on pfrom
//        for (int itearation = 0;itearation<10 && pfrom->distance(P)>0.0001;itearation++) {
//            P = pfrom->getPosition();
//            pdest = edest->project(P);
//            pfrom = it_element.efrom->project(pdest->getPosition());
//        }

        //compute all the distances with to elements
        defaulttype::Vector3 N = P - pdest->getPosition();
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

    if (min_pair.first == nullptr) return;
    if (min_pair.second == nullptr) return;

    addDetectionOutput(min_pair.first,min_pair.second);
}

BaseElement::Iterator CollisionDetectionAlgorithm::selectElementsOnDest(const defaulttype::Vector3 & P) const {
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


void CollisionDetectionAlgorithm::processAlgorithm()
{
//    AABBDecorator * from = NULL;
//    BaseElementFilter* filter = l_filter.get();

//    //first we search if there is a AABB connected to the geometry
//    for (unsigned i=0;i<p_from->p_type.size();i++) {
//        if ((from = dynamic_cast<AABBDecorator *>(p_from->p_type[i]))) break;
//    }

//    if(!filter)
//        filter = new DefaultElementFilter(l_dest.get());

    //we do the collision from first to second
    for (auto it=l_from->begin();it!=l_from->end();it++) {
        BaseProximity::SPtr pfrom = it->center();

        BaseElement::Iterator itdest = selectElementsOnDest(pfrom->getPosition());


        findClosestPoint(pfrom, itdest);
    }

//    delete filter;
//    //then from second to first
//    for (unsigned i=0;i<p_dest->getNbElements();i++) {
//        BaseElementPtr elmt = p_dest->getElement(i);
//        PairProximity pair = (from == NULL) ? getClosestPoint(DefaultIterator(elmt, p_from())) : getClosestPoint(AABBElementIterator(elmt, from));

//        if (pair.first == NULL) continue;
//        if (pair.second == NULL) continue;

//        m_pairDetection.push_back(PairProximity(pair.second,pair.first));
//    }
}

}

}

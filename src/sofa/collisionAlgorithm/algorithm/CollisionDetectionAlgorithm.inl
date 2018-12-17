#pragma once

#include <sofa/collisionAlgorithm/algorithm/CollisionDetectionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElementFilter.h>
#include <limits>


namespace sofa
{

namespace collisionAlgorithm
{

template<class ElementIterator>
PairProximity CollisionDetectionAlgorithm::getClosestPoint(std::unique_ptr<ElementIterator> it_element)
{
    PairProximity min_pair;
    double min_dist = std::numeric_limits<double>::max();
    min_pair.first = nullptr;
    min_pair.second = nullptr;

    ConstraintProximity::SPtr pfrom = it_element->getFrom()->getControlPoint(); //centered control point
    if (pfrom == nullptr)
        return min_pair;

    defaulttype::Vector3 P = pfrom->getPosition();

    for (unsigned i=0;i<it_element->size();i++)
    {
        const ConstraintElement * edest = it_element->element(i);
        ConstraintProximity::SPtr pdest = edest->project(P);

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
        if (dot(pfrom->getNormal(),pdest->getNormal()) > -d_minAngle.getValue())
            continue;

        if (dist<min_dist)
        {
            min_dist = dist;
            min_pair.first = pfrom;
            min_pair.second = pdest;
        }
    }

    return min_pair;
}

void CollisionDetectionAlgorithm::processAlgorithm()
{
    this->m_pairDetection.clear();

//    AABBDecorator * from = NULL;
    BaseElementFilter* filter = l_filter.get();

//    //first we search if there is a AABB connected to the geometry
//    for (unsigned i=0;i<p_from->p_type.size();i++) {
//        if ((from = dynamic_cast<AABBDecorator *>(p_from->p_type[i]))) break;
//    }

    if(!filter)
        filter = new DefaultElementFilter(l_dest.get());

    //we do the collision from first to second
    for (unsigned i=0;i<l_from->getNbElements();i++)
    {
        const ConstraintElement * elmt = l_from->getElement(i);
        PairProximity pair = getClosestPoint(filter->iterator(elmt));

        if (pair.first == nullptr)
            continue;
        if (pair.second == nullptr)
            continue;

        m_pairDetection.push_back(PairProximity(pair.first,pair.second));
    }

    delete filter;
//    //then from second to first
//    for (unsigned i=0;i<p_dest->getNbElements();i++) {
//        ConstraintElementPtr elmt = p_dest->getElement(i);
//        PairProximity pair = (from == NULL) ? getClosestPoint(DefaultIterator(elmt, p_from())) : getClosestPoint(AABBElementIterator(elmt, from));

//        if (pair.first == NULL) continue;
//        if (pair.second == NULL) continue;

//        m_pairDetection.push_back(PairProximity(pair.second,pair.first));
//    }
}

}

}

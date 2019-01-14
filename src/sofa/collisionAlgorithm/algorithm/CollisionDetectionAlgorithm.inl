#pragma once

#include <sofa/collisionAlgorithm/algorithm/CollisionDetectionAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElementFilter.h>
#include <limits>


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

BaseElement::Iterator CollisionDetectionAlgorithm::selectElements(const defaulttype::Vector3 & P, BaseDecorator * decorator) {
    //compute the box where is P
    defaulttype::Vec3i cbox;
    cbox[0] = floor((P[0] - m_Bmin[0])/m_cellSize[0]);
    cbox[1] = floor((P[1] - m_Bmin[1])/m_cellSize[1]);
    cbox[2] = floor((P[2] - m_Bmin[2])/m_cellSize[2]);

    //project the box in the bounding box of the object
    //search with the closest box in bbox
    for (unsigned int i=0;i<3;i++)
    {
        if (cbox[i] < 0)
            cbox[i] = 0;
        else
        {
            if (cbox[i] > m_nbox[i])
                cbox[i] = m_nbox[i];
        }
    }

    int d = 0;
    int max = std::max(std::max(m_nbox[0],m_nbox[1]),m_nbox[2]);

    while (selectElements.empty() && d<max)
    {
        fillElementSet(cbox,d,selectElements);
        d++;// we look for boxed located at d+1
    }
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

        BaseElement::Iterator itdest = selectElements(pfrom->getPosition(), l_dest->getBroadPhase());


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

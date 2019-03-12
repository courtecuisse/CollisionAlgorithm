#pragma once

#include <sofa/collisionAlgorithm/algorithm/FindClosestPointAlgorithm.h>
#include <sofa/collisionAlgorithm/iterators/SubsetElementIterator.h>
#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>

namespace sofa
{

namespace collisionAlgorithm
{

void BaseClosestPointAlgorithm::fillElementSet(const BroadPhase * decorator, defaulttype::Vec3i cbox, std::set<unsigned> & selectElements, int d) const
{
    defaulttype::Vec3i nbox = decorator->getBoxSize();

    {
        int i=-d;
        if (cbox[0]+i >= 0 && cbox[0]+i < nbox[0])
        {
            for (int j=-d;j<=d;j++)
            {
                if (cbox[1]+j < 0 || cbox[1]+j >= nbox[1])
                    continue;
                for (int k=-d;k<=d;k++)
                {
                    if (cbox[2]+k < 0 || cbox[2]+k >= nbox[2])
                        continue;

                    decorator->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k, selectElements);
                }
            }
        }
    }

    {
        int i=d;
        if (cbox[0]+i >= 0 && cbox[0]+i < nbox[0])
        {
            for (int j=-d;j<=d;j++)
            {
                if (cbox[1]+j < 0 || cbox[1]+j >= nbox[1])
                    continue;

                for (int k=-d;k<=d;k++)
                {
                    if (cbox[2]+k < 0 || cbox[2]+k >= nbox[2])
                        continue;

                    decorator->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k, selectElements);
                }
            }
        }
    }


    {
        int j=-d;
        if (cbox[1]+j >= 0 && cbox[1]+j < nbox[1])
        {
            for (int i=-d+1;i<d;i++)
            {
                if (cbox[0]+i < 0 || cbox[0]+i >= nbox[0])
                    continue;

                for (int k=-d;k<=d;k++)
                {
                    if (cbox[2]+k < 0 || cbox[2]+k >= nbox[2])
                        continue;

                    decorator->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k, selectElements);
                }
            }
        }
    }

    {
        int j=d;
        if (cbox[1]+j >= 0 && cbox[1]+j < nbox[1])
        {
            for (int i=-d+1;i<d;i++)
            {
                if (cbox[0]+i < 0 || cbox[0]+i >= nbox[0])
                    continue;

                for (int k=-d;k<=d;k++)
                {
                    if (cbox[2]+k < 0 || cbox[2]+k >= nbox[2])
                        continue;

                    decorator->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k, selectElements);
                }
            }
        }
    }

    {
        int k=-d;
        if (cbox[2]+k >= 0 && cbox[2]+k < nbox[2])
        {
            for (int i=-d+1;i<d;i++)
            {
                if (cbox[0]+i < 0 || cbox[0]+i >= nbox[0])
                    continue;

                for (int j=-d+1;j<d;j++)
                {
                    if (cbox[1]+j < 0 || cbox[1]+j >= nbox[1])
                        continue;

                    decorator->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k, selectElements);
                }
            }
        }
    }

    {
        int k=d;
        if (cbox[2]+k >= 0 && cbox[2]+k < nbox[2])
        {
            for (int i=-d+1;i<d;i++)
            {
                if (cbox[0]+i < 0 || cbox[0]+i >= nbox[0])
                    continue;

                for (int j=-d+1;j<d;j++)
                {
                    if (cbox[1]+j < 0 || cbox[1]+j >= nbox[1])
                        continue;

                    decorator->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k, selectElements);
                }
            }
        }
    }
}

BaseElementIterator::UPtr BaseClosestPointAlgorithm::getDestIterator(const defaulttype::Vector3 & P, const BaseGeometry * geo) {
    const BroadPhase * decorator = geo->getBroadPhase();

    if (decorator == NULL) return geo->begin();
    else {
        defaulttype::Vec3i bindex = decorator->getBoxCoord(P);
        defaulttype::Vec3i bsize = decorator->getBoxSize();

        int max = 0;
        max = std::max(max,bindex[0]);
        max = std::max(max,bindex[1]);
        max = std::max(max,bindex[2]);
        max = std::max(max,bsize[0]-bindex[0]);
        max = std::max(max,bsize[1]-bindex[1]);
        max = std::max(max,bsize[2]-bindex[2]);

        int d = 0;
        std::set<unsigned> selectedElements;
        while (selectedElements.empty() && d<max) {
            fillElementSet(decorator,bindex,selectedElements,d);
            d++;// we look for boxed located at d+1
        }

        return BaseElementIterator::UPtr(new SubsetElementIterator(geo, selectedElements));
    }
}

DetectionOutput::PairDetection BaseClosestPointAlgorithm::findClosestPoint(const BaseElement::UPtr & elfrom, const BaseGeometry * geo)
{
    double min_dist = std::numeric_limits<double>::max();
    DetectionOutput::PairDetection min_pair(nullptr,nullptr);

    defaulttype::Vector3 P = elfrom->center()->getPosition();

    BaseElementIterator::UPtr itdest=getDestIterator(P,geo);

    while (itdest != geo->end())
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

BaseProximity::SPtr BaseClosestPointAlgorithm::findClosestPoint(BaseProximity::SPtr pfrom, BaseElementIterator::UPtr itdest) {
    double min_dist = std::numeric_limits<double>::max();
    BaseProximity::SPtr minprox = nullptr;

    defaulttype::Vector3 P = pfrom->getPosition();

    while (! itdest->end())
    {
        BaseProximity::SPtr pdest = (*itdest)->project(P);

        if (acceptFilter(pfrom,pdest)) {
            defaulttype::Vector3 N = P - pdest->getPosition();
            double dist = N.norm();

            if (dist<min_dist) {
                min_dist = dist;
                minprox = pdest;
            }
        }

        itdest++;
    }

    return minprox;
}


BaseProximity::SPtr BaseClosestPointAlgorithm::findClosestPoint(BaseProximity::SPtr pfrom, const BaseGeometry * geo) {
    return findClosestPoint(pfrom,std::move(getDestIterator(pfrom->getPosition(),geo)));
}

void FindClosestPointAlgorithm::doDetection() {
    if (l_from == NULL) return;
    if (l_dest == NULL) return;

    DetectionOutput & output = *d_output.beginEdit();
    output.clear();
    for (auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++) {
        DetectionOutput::PairDetection min_pair = findClosestPoint(*itfrom,l_dest.get());

        if (min_pair.first == nullptr || min_pair.second == nullptr) continue;

        output.add(min_pair.first,min_pair.second);
    }
    d_output.endEdit();
}

}

}

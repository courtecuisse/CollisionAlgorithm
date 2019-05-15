#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

#include <sofa/collisionAlgorithm/data/DataDistanceMeasure.h>
#include <sofa/collisionAlgorithm/iterators/SubsetElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BaseClosestProximityAlgorithm : public BaseAlgorithm
{
public:
    SOFA_CLASS(BaseClosestProximityAlgorithm, BaseAlgorithm);

    Data<unsigned> d_iterations;
    Data<DistanceMeasure> d_distance_measure;

    static double norme3(const collisionAlgorithm::PairDetection & d) {
        return (d.first->getPosition()-d.second->getPosition()).norm() ;
    }

    BaseClosestProximityAlgorithm ()
    : d_iterations(initData(&d_iterations,(unsigned) 1,"iterations", "numberof iterations"))
    , d_distance_measure(initData(&d_distance_measure,
                                  DistanceMeasure(std::bind(&norme3, std::placeholders::_1)), "distance", "distance measure component"))
    {}

    void fillElementSet(const BaseGeometry::BroadPhase::SPtr decorator, defaulttype::Vec3i cbox, std::set<unsigned> & selectElements, int d) const
    {
        defaulttype::Vec3i nbox = decorator->getBoxSize();

        {
            int i=-d;
            if (cbox[0]+i >= 0 && cbox[0]+i < nbox[0])
            {
                for (int j=-d; j <= d; j++)
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

    BaseElementIterator::UPtr getDestIterator(const defaulttype::Vector3 & P, BaseGeometry *geo) {
        const std::vector<BaseGeometry::BroadPhase::SPtr> & decorators = geo->getBroadPhase();

        if (decorators.empty()) return geo->begin();
        else {
            //take the first broad phase...
            const BaseGeometry::BroadPhase::SPtr & decorator = decorators[0];

            defaulttype::Vec3i bindex = decorator->getBoxCoord(P);
            defaulttype::Vec3i bsize = decorator->getBoxSize();

            int max = 0;

            for (int i = 0 ; i < 3 ; i++) {
                max = std::max (max, bindex[i]) ;
                max = std::max (max, bsize[i]-bindex[i]) ;
            }
    //        max = std::max(max,bindex[0]);
    //        max = std::max(max,bindex[1]);
    //        max = std::max(max,bindex[2]);
    //        max = std::max(max,bsize[0]-bindex[0]);
    //        max = std::max(max,bsize[1]-bindex[1]);
    //        max = std::max(max,bsize[2]-bindex[2]);

            int d = 0;
            std::set<unsigned> selectedElements;
            while (selectedElements.empty() && d<max) {
                fillElementSet(decorator,bindex,selectedElements,d);
                d++;// we look for boxed located at d+1
            }

            return BaseElementIterator::UPtr(new SubsetElementIterator(geo, selectedElements));
        }
    }

    PairDetection findClosestPoint(const BaseElementIterator *itfrom,  BaseGeometry *geo) {
        double min_dist = std::numeric_limits<double>::max();
        BaseProximity::SPtr minprox_from = nullptr;
        BaseProximity::SPtr minprox_dest = nullptr;

        for(BaseElementIterator::UPtr itdest = getDestIterator(itfrom->center()->getPosition(),geo); // this function may create an iterator on using the bradphase if defined in the geometry
            itdest != geo->end(); itdest++)
        {
            BaseProximity::SPtr pfrom = itfrom->center();
            BaseProximity::SPtr pdest = itdest->project(pfrom->getPosition());

            // Internal iterations to reproject on the from element
            // For linear elements (triangles, edges, ...) it = 1 should be sufficient
            // For points (from) it can be set to 0 since the reprojection will not modify the proximity
            // For non linear elements (bezier, ...) few iterations may be necessary
            for (unsigned it=0;it<d_iterations.getValue();it++) {
                pfrom = itfrom->project(pdest->getPosition());
                pdest = itdest->project(pfrom->getPosition());
            }

            if (acceptFilter(pfrom,pdest)) {
                double dist = d_distance_measure.getValue().compute(PairDetection(pfrom,pdest));

                if (dist<min_dist) {
                    min_dist = dist;
                    minprox_dest = pdest;
                    minprox_from = pfrom;
                }
            }
        }

        return PairDetection(minprox_from,minprox_dest);
    }


    PairDetection findClosestPoint(const BaseProximity::SPtr pfrom,  BaseGeometry *geo) {
        double min_dist = std::numeric_limits<double>::max();
        BaseProximity::SPtr minprox_dest = nullptr;

        for(BaseElementIterator::UPtr itdest = getDestIterator(pfrom->getPosition(),geo); // this function may create an iterator on using the bradphase if defined in the geometry
            itdest != geo->end(); itdest++)
        {
            BaseProximity::SPtr pdest = itdest->project(pfrom->getPosition());

            if (acceptFilter(pfrom,pdest)) {
                double dist = d_distance_measure.getValue().compute(PairDetection(pfrom,pdest));

                if (dist<min_dist) {
                    min_dist = dist;
                    minprox_dest = pdest;
                }
            }
        }

        return PairDetection(pfrom,minprox_dest);
    }

};


}

}

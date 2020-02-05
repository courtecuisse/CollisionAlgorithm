#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/SubsetElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

//Specific class to change the distance between proximities
class BaseDistanceProximityMeasure : public sofa::core::objectmodel::BaseObject {
public :
    SOFA_ABSTRACT_CLASS(BaseDistanceProximityMeasure, sofa::core::objectmodel::BaseObject);

    virtual double computeDistance(const collisionAlgorithm::PairDetection & d) const = 0;

} ;

//Default implementation of a 3D distance
class Distance3DProximityMeasure : public BaseDistanceProximityMeasure {
public :
    SOFA_CLASS(Distance3DProximityMeasure, BaseDistanceProximityMeasure);

    double computeDistance(const collisionAlgorithm::PairDetection & d) const override {
        return (d.first->getPosition()-d.second->getPosition()).norm() ;
    }
} ;

//Static functions
void fillElementSet(const BaseGeometry::BroadPhase::SPtr decorator, defaulttype::Vec3i cbox, std::set<unsigned> & selectElements, int d);
BaseProximity::SPtr doFindClosestProximityIt(const BaseProximity::SPtr & pfrom, BaseElementIterator::UPtr & begin, std::function<bool(const BaseProximity::SPtr, const BaseProximity::SPtr)>& acceptFilter, const BaseDistanceProximityMeasure& distance );
BaseProximity::SPtr findClosestProximity(const BaseProximity::SPtr & pfrom, BaseGeometry *geo, std::function<bool(const BaseProximity::SPtr, const BaseProximity::SPtr)>& acceptFilter, const BaseDistanceProximityMeasure& distance );

class BaseClosestProximityAlgorithm : public BaseAlgorithm
{
public:
    SOFA_CLASS(BaseClosestProximityAlgorithm, BaseAlgorithm);

    Data<unsigned> d_iterations;
    Data<double> d_threshold;
    core::objectmodel::SingleLink<BaseClosestProximityAlgorithm,BaseDistanceProximityMeasure, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_distance;

    BaseClosestProximityAlgorithm ()
    : d_iterations(initData(&d_iterations,(unsigned) 1,"iterations", "Number of reprojections of between pair of elements"))
    , d_threshold(initData(&d_threshold,0.0000001,"threshold", "Threshold for iterations"))
    , l_distance(initLink("distance", "link to the compoenent that computes the distance between proximities"))
    {}

    void init() { // make sure we have a direction
        if (this->l_distance == NULL) l_distance = sofa::core::objectmodel::New<Distance3DProximityMeasure>();
        l_distance->setName("defaultDistance");
        this->addSlave(l_distance.get());
    }

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

    BaseProximity::SPtr doFindClosestProximityIt(const BaseProximity::SPtr & pfrom, BaseElementIterator::UPtr & begin) {
        double min_dist = std::numeric_limits<double>::max();
        BaseProximity::SPtr minprox_dest = nullptr;
        defaulttype::Vector3 P = pfrom->getPosition();

        while (! begin->end()) {
            BaseProximity::SPtr pdest = begin->project(P);

            if (acceptFilter(pfrom,pdest)) {
                double dist = l_distance->computeDistance(PairDetection(pfrom,pdest));

                if (dist<min_dist) {
                    min_dist = dist;
                    minprox_dest = pdest;
                }
            }

            begin++;
        }

        return minprox_dest;
    }


    BaseProximity::SPtr findClosestProximity(const BaseProximity::SPtr & pfrom, BaseGeometry *geo) {
        const std::vector<BaseGeometry::BroadPhase::SPtr> & decorators = geo->getBroadPhase();

        if (decorators.empty()) {
            BaseElementIterator::UPtr begin = geo->begin();
            return doFindClosestProximityIt(pfrom,begin);
        } else {
            //take the first broad phase...
            const BaseGeometry::BroadPhase::SPtr & decorator = decorators.front();
            defaulttype::Vector3 P = pfrom->getPosition();

            defaulttype::Vec3i bindex = decorator->getBoxCoord(P);
            defaulttype::Vec3i bsize = decorator->getBoxSize();

            int max = 0;

            for (int i = 0 ; i < 3 ; i++) {
                max = std::max (max, bindex[i]) ;
                max = std::max (max, bsize[i]-bindex[i]) ;
            }

            BaseProximity::SPtr minprox_dest = nullptr;
            int d = 0;
            std::set<unsigned> selectedElements;
            while (selectedElements.empty() && d<max) {
                fillElementSet(decorator,bindex,selectedElements,d);

                BaseElementIterator::UPtr begin(new SubsetElementIterator(geo, selectedElements));
                minprox_dest = doFindClosestProximityIt(pfrom, begin);

                d++;// we look for boxed located at d+1

                //take the first on that satisfy filters
                if (minprox_dest != NULL) return minprox_dest;
            }
        }

        return NULL;
    }

    PairDetection findClosestPoint(const BaseElementIterator *itfrom, BaseGeometry *geo) {
        BaseProximity::SPtr pfrom = itfrom->createProximity();
        defaulttype::Vector3 prevPos = pfrom->getPosition();
        BaseProximity::SPtr pdest = findClosestProximity(pfrom,geo);

        if (pfrom == NULL || pdest == NULL) return PairDetection(pfrom,pdest);

        // Internal iterations to reproject on the from element
        // For linear elements (triangles, edges, ...) it = 1 should be sufficient
        // For points (from) it can be set to 0 since the reprojection will not modify the proximity
        // For non linear elements (bezier, ...) few iterations may be necessary
        for (unsigned it=0;it<d_iterations.getValue();it++) {
            pfrom = itfrom->project(pdest->getPosition()); // reproject on pfrom
            if ((prevPos - pfrom->getPosition()).norm() < d_threshold.getValue()) break;

            pfrom = itfrom->createProximity();
            pdest = findClosestProximity(pfrom,geo);

            if (pfrom == NULL || pdest == NULL) return PairDetection(pfrom,pdest);
        }


        return PairDetection(pfrom,pdest);
    }


    PairDetection findClosestPoint(const BaseProximity::SPtr & pfrom, BaseGeometry *geo) {
        BaseProximity::SPtr pdest = findClosestProximity(pfrom,geo);
        return PairDetection(pfrom,pdest);
    }

};


}

}

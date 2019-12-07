#include <sofa/collisionAlgorithm/algorithm/BaseClosestProximityAlgorithm.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

//Static functions
void fillElementSet(const BaseGeometry::BroadPhase::SPtr decorator, defaulttype::Vec3i cbox, std::set<unsigned> & selectElements, int d)
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

BaseProximity::SPtr doFindClosestProximityIt(const BaseProximity::SPtr & pfrom, BaseElementIterator::UPtr & begin, std::function<bool(const BaseProximity::SPtr, const BaseProximity::SPtr)>& acceptFilter, const BaseDistanceProximityMeasure& distance ) {
    double min_dist = std::numeric_limits<double>::max();
    BaseProximity::SPtr minprox_dest = nullptr;
    defaulttype::Vector3 P = pfrom->getPosition();

    while (! begin->end()) {
        BaseProximity::SPtr pdest = begin->project(P);

        if (acceptFilter(pfrom,pdest)) {
            double dist = distance.computeDistance(PairDetection(pfrom,pdest));

            if (dist<min_dist) {
                min_dist = dist;
                minprox_dest = pdest;
            }
        }

        begin++;
    }

    return minprox_dest;
}

BaseProximity::SPtr findClosestProximity(const BaseProximity::SPtr & pfrom, BaseGeometry *geo, std::function<bool(const BaseProximity::SPtr, const BaseProximity::SPtr)>& acceptFilter, const BaseDistanceProximityMeasure& distance ) {
    const std::vector<BaseGeometry::BroadPhase::SPtr> & decorators = geo->getBroadPhase();

    if (decorators.empty()) {
        BaseElementIterator::UPtr begin = geo->begin();
        return doFindClosestProximityIt(pfrom,begin,acceptFilter,distance);
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
            minprox_dest = doFindClosestProximityIt(pfrom, begin, acceptFilter, distance);

            d++;// we look for boxed located at d+1

            //take the first on that satisfy filters
            if (minprox_dest != NULL) return minprox_dest;
        }
    }

    return NULL;
}

}

}




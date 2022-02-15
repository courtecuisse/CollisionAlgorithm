#include <sofa/collisionAlgorithm/algorithm/FindClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/geometry/AABBGeometry.h>
#include <sofa/collisionAlgorithm/iterators/SubsetElementIterator.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(FindClosestProximityAlgorithm)

int FindClosestPointAlgorithmClass = core::RegisterObject("FindClosestProximityAlgorithm")
.add< FindClosestProximityAlgorithm >();


static BaseProximity::SPtr doFindClosestProx(BaseProximity::SPtr prox, ElementIterator::SPtr itdest,
                                             Operations::ProjectOperation::FUNC projectOp,
                                             std::function<double(BaseProximity::SPtr,BaseProximity::SPtr)> distance) {
    double min_dist = std::numeric_limits<double>::max();
    BaseProximity::SPtr res = NULL;

    while (! itdest->end()) {
        auto edest = itdest->element();
        if (edest == nullptr) continue;

        BaseProximity::SPtr pdest = projectOp(prox->getPosition(),edest);

        double d = distance(prox,pdest);
        if (d < min_dist) {
            res = pdest;
            min_dist = d;
        }

        itdest++;
    }

    return res;
}

//By default no broadPhase so we loop over all elements
static BaseProximity::SPtr genericFindClosestPoint(BaseProximity::SPtr prox,BaseGeometry * geo,Operations::ProjectOperation::FUNC projectOp,std::function<double(BaseProximity::SPtr,BaseProximity::SPtr)> distance) {
    return doFindClosestProx(prox,geo->begin(),projectOp,distance);
}

//By default no broadPhase so we loop over all elements
FindClosestProximityOperation::GenericOperation::FUNC FindClosestProximityOperation::getDefault() const {
    return &genericFindClosestPoint;
}

template<class BROADPHASE>
static BaseProximity::SPtr FindClosestProximityOperationWithAABB(BaseProximity::SPtr prox, BaseGeometry * geo,
                                                                 Operations::ProjectOperation::FUNC projectOp,
                                                                 std::function<double(BaseProximity::SPtr,BaseProximity::SPtr)> distance) {
    BROADPHASE * broadphase = (BROADPHASE*) geo;

    //old params : type::Vec3i cbox, std::set<BaseProximity::Index> & selectElements, int d
    type::Vec3i nbox = broadphase->getBoxSize();

    //take the first broad phase...
    type::Vector3 P = prox->getPosition();

    type::Vec3i cbox = broadphase->getBoxCoord(P);

    int max = 0;
    for (int i = 0 ; i < 3 ; i++) {
        max = std::max (max, cbox[i]) ;
        max = std::max (max, nbox[i]-cbox[i]) ;
    }

    int d = 0;
    std::set<BaseProximity::Index> selectedElements;
    while (selectedElements.empty() && d<max) {
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

                        broadphase->getElementSet(type::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectedElements);
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

                        broadphase->getElementSet(type::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectedElements);
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

                        broadphase->getElementSet(type::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectedElements);
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

                        broadphase->getElementSet(type::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectedElements);
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

                        broadphase->getElementSet(type::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectedElements);
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

                        broadphase->getElementSet(type::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectedElements);
                    }
                }
            }
        }

        d++;// we look for boxed located at d+1
    }

    ElementIterator::SPtr it_subset = ElementIterator::SPtr(new SubsetElementIterator(geo,selectedElements));
    return doFindClosestProx(prox,it_subset,projectOp,distance);
}

int register_FindClosestProximityOperationAABB = FindClosestProximityOperation::register_func<AABBGeometry>(&FindClosestProximityOperationWithAABB<AABBGeometry>);

}

}




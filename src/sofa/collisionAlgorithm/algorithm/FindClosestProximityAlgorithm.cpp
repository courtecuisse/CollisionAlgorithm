#include <sofa/collisionAlgorithm/algorithm/FindClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/geometry/AABBGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(FindClosestProximityAlgorithm)

int FindClosestPointAlgorithmClass = core::RegisterObject("FindClosestProximityAlgorithm")
.add< FindClosestProximityAlgorithm >();


static BaseProximity::SPtr doFindClosestProx(BaseProximity::SPtr prox, ElementIterator::SPtr itdest) {
    double min_dist = std::numeric_limits<double>::max();
    BaseProximity::SPtr res = NULL;

    type::Vector3 P = prox->getPosition();
    Operations::ProjectOperation::FUNC projectOp = Operations::ProjectOperation::get(itdest->getOperationsHash());

    for (; ! itdest->end();itdest++) {
        auto edest = itdest->element();
        if (edest == nullptr) continue;

        BaseProximity::SPtr pdest = projectOp(prox->getPosition(),edest);
        if (pdest == NULL) continue;

        double d = (P - pdest->getPosition()).norm();
        if (d < min_dist) {
            res = pdest;
            min_dist = d;
        }
    }

    return res;
}

//By default no broadPhase so we loop over all elements
static BaseProximity::SPtr genericFindClosestPoint(BaseProximity::SPtr prox,BaseGeometry * geo) {
    return doFindClosestProx(prox,geo->begin());
}

//By default no broadPhase so we loop over all elements
FindClosestProximityOperation::GenericOperation::FUNC FindClosestProximityOperation::getDefault() const {
    return &genericFindClosestPoint;
}

template<class BROADPHASE>
static BaseProximity::SPtr FindClosestProximityOperationWithAABB(BaseProximity::SPtr prox, BaseGeometry * geo) {
    BROADPHASE * broadphase = (BROADPHASE*) geo;

    //old params : type::Vec3i cbox, std::set<BaseProximity::Index> & selectElements, int d
    type::Vec3i nbox = broadphase->d_nbox.getValue();

    //take the first broad phase...
    type::Vector3 P = prox->getPosition();

    type::Vec3i cbox = broadphase->getBoxCoord(P);

    //project the box in the bounding box of the object
    //search with the closest box in bbox
    cbox[0] = std::max(0,std::min(nbox[0]-1,cbox[0]));
    cbox[1] = std::max(0,std::min(nbox[1]-1,cbox[1]));
    cbox[2] = std::max(0,std::min(nbox[2]-1,cbox[2]));

    int max = 0;
    for (int i = 0 ; i < 3 ; i++) {
        max = std::max (max, cbox[i]) ;
        max = std::max (max, nbox[i]-cbox[i]) ;
    }

    int d = 0;
    std::set<BaseElement * > selectedElements;

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

                        std::set<BaseElement*> elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        selectedElements.insert(elmts.cbegin(),elmts.cend());
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

                        std::set<BaseElement*> elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        selectedElements.insert(elmts.cbegin(),elmts.cend());
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

                        std::set<BaseElement*> elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        selectedElements.insert(elmts.cbegin(),elmts.cend());
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

                        std::set<BaseElement*> elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        selectedElements.insert(elmts.cbegin(),elmts.cend());
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

                        std::set<BaseElement*> elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        selectedElements.insert(elmts.cbegin(),elmts.cend());
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

                        std::set<BaseElement*> elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        selectedElements.insert(elmts.cbegin(),elmts.cend());
                    }
                }
            }
        }

        d++;// we look for boxed located at d+1
    }

    return doFindClosestProx(prox,ElementIterator::SPtr(new TDefaultElementIteratorPtr(selectedElements)));
}

int register_FindClosestProximityOperationAABB = FindClosestProximityOperation::register_func<AABBGeometry::AABBBElement>(&FindClosestProximityOperationWithAABB<AABBGeometry>);

}

}




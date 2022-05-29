#include <sofa/collisionAlgorithm/algorithm/FindClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/geometry/AABBGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(FindClosestProximityAlgorithm)

int FindClosestPointAlgorithmClass = core::RegisterObject("FindClosestProximityAlgorithm")
.add< FindClosestProximityAlgorithm >();




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
    std::set<BaseElement::SPtr> selectedElements;

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

                        const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        std::cout << "C0 " << cbox[0] + i << " , " << cbox[1] + j << " , " << cbox[2] + k << " :" << elmts.size() << std::endl;
                        selectedElements.insert(elmts.cbegin(),elmts.cend());
                    }
                }
            }
        }

        //No need to process all the box for d==0 since they all point on the same box
        if (d==0) continue;

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

                        const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        std::cout << "C1 : " << elmts.size() << std::endl;
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

                        const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        std::cout << "C2 : " << elmts.size() << std::endl;
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

                        const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        std::cout << "C3 : " << elmts.size() << std::endl;
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

                        const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        std::cout << "C4 : " << elmts.size() << std::endl;
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

                        const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        std::cout << "C5 : " << elmts.size() << std::endl;
                        selectedElements.insert(elmts.cbegin(),elmts.cend());
                    }
                }
            }
        }

        d++;// we look for boxed located at d+1
    }

    ElementIterator::SPtr iterator(new TDefaultElementIteratorPtr(selectedElements));
    return FindClosestProximityOperation::doFindClosestProx(prox,iterator);
}

int register_FindClosestProximityOperationAABB = FindClosestProximityOperation::register_func<AABBGeometry::AABBBElement>(&FindClosestProximityOperationWithAABB<AABBGeometry>);

}

}




#include <sofa/collisionAlgorithm/algorithm/FindClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/geometry/AABBBroadPhaseGeometry.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(FindClosestProximityAlgorithm)

int FindClosestPointAlgorithmClass = core::RegisterObject("FindClosestProximityAlgorithm")
.add< FindClosestProximityAlgorithm >();

static BaseElement::Iterator broadPhase(type::Vector3 P, BaseGeometry * geometry) {
//    //old params : type::Vec3i cbox, std::set<BaseProximity::Index> & selectElements, int d
//    type::Vec3i nbox = decorator->getBoxSize();

//    {
//        int i=-d;
//        if (cbox[0]+i >= 0 && cbox[0]+i < nbox[0])
//        {
//            for (int j=-d; j <= d; j++)
//            {
//                if (cbox[1]+j < 0 || cbox[1]+j >= nbox[1])
//                    continue;
//                for (int k=-d;k<=d;k++)
//                {
//                    if (cbox[2]+k < 0 || cbox[2]+k >= nbox[2])
//                        continue;

//                    decorator->getElementSet(type::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectElements);
//                }
//            }
//        }
//    }

//    {
//        int i=d;
//        if (cbox[0]+i >= 0 && cbox[0]+i < nbox[0])
//        {
//            for (int j=-d;j<=d;j++)
//            {
//                if (cbox[1]+j < 0 || cbox[1]+j >= nbox[1])
//                    continue;

//                for (int k=-d;k<=d;k++)
//                {
//                    if (cbox[2]+k < 0 || cbox[2]+k >= nbox[2])
//                        continue;

//                    decorator->getElementSet(type::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectElements);
//                }
//            }
//        }
//    }


//    {
//        int j=-d;
//        if (cbox[1]+j >= 0 && cbox[1]+j < nbox[1])
//        {
//            for (int i=-d+1;i<d;i++)
//            {
//                if (cbox[0]+i < 0 || cbox[0]+i >= nbox[0])
//                    continue;

//                for (int k=-d;k<=d;k++)
//                {
//                    if (cbox[2]+k < 0 || cbox[2]+k >= nbox[2])
//                        continue;

//                    decorator->getElementSet(type::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectElements);
//                }
//            }
//        }
//    }

//    {
//        int j=d;
//        if (cbox[1]+j >= 0 && cbox[1]+j < nbox[1])
//        {
//            for (int i=-d+1;i<d;i++)
//            {
//                if (cbox[0]+i < 0 || cbox[0]+i >= nbox[0])
//                    continue;

//                for (int k=-d;k<=d;k++)
//                {
//                    if (cbox[2]+k < 0 || cbox[2]+k >= nbox[2])
//                        continue;

//                    decorator->getElementSet(type::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectElements);
//                }
//            }
//        }
//    }

//    {
//        int k=-d;
//        if (cbox[2]+k >= 0 && cbox[2]+k < nbox[2])
//        {
//            for (int i=-d+1;i<d;i++)
//            {
//                if (cbox[0]+i < 0 || cbox[0]+i >= nbox[0])
//                    continue;

//                for (int j=-d+1;j<d;j++)
//                {
//                    if (cbox[1]+j < 0 || cbox[1]+j >= nbox[1])
//                        continue;

//                    decorator->getElementSet(type::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectElements);
//                }
//            }
//        }
//    }

//    {
//        int k=d;
//        if (cbox[2]+k >= 0 && cbox[2]+k < nbox[2])
//        {
//            for (int i=-d+1;i<d;i++)
//            {
//                if (cbox[0]+i < 0 || cbox[0]+i >= nbox[0])
//                    continue;

//                for (int j=-d+1;j<d;j++)
//                {
//                    if (cbox[1]+j < 0 || cbox[1]+j >= nbox[1])
//                        continue;

//                    decorator->getElementSet(type::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectElements);
//                }
//            }
//        }
//    }
}

int register_FindClosestProximityAlgorithm_BroadPhase = FindClosestProximityAlgorithm_BroadPhase::register_func<AABBBroadPhaseGeometry::AABBBElement>(&broadPhase);

}

}




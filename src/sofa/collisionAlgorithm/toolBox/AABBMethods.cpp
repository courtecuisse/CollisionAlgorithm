#include <sofa/collisionAlgorithm/toolBox/ClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/toolBox/AABBMethods.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/collisionAlgorithm/geometry/SubsetGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

namespace toolBox {

//void fillElementSet(const BaseGeometry::BroadPhase::SPtr decorator, type::Vec3i cbox, std::set<BaseProximity::Index> & selectElements, int d) {
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
//}

//BaseProximity::SPtr findClosestProximity(const BaseProximity::SPtr & pfrom, BaseGeometry *geo,
//                                         std::function<bool(const collisionAlgorithm::PairDetection & )> acceptFilter,
//                                         std::function<double(const collisionAlgorithm::PairDetection & )> distance )
//{
//    BaseGeometry::BroadPhase::SPtr decorator = geo->getBroadPhase();

//    if (decorator == NULL) {
//        BaseElementIterator::UPtr begin = geo->begin();
//        return toolBox::doFindClosestProximityIt(pfrom,begin,
//                                                 acceptFilter,
//                                                 distance);
//    } else {
//        //take the first broad phase...
//        type::Vector3 P = pfrom->getPosition();

//        type::Vec3i bindex = decorator->getBoxCoord(P);
//        type::Vec3i bsize = decorator->getBoxSize();

//        int max = 0;

//        for (int i = 0 ; i < 3 ; i++) {
//            max = std::max (max, bindex[i]) ;
//            max = std::max (max, bsize[i]-bindex[i]) ;
//        }

//        BaseProximity::SPtr minprox_dest = nullptr;
//        int d = 0;
//        std::set<BaseProximity::Index> selectedElements;
//        while (selectedElements.empty() && d<max) {
//            fillElementSet(decorator,bindex,selectedElements,d);

//            BaseElementIterator::UPtr begin(new SubsetElementIterator(geo, selectedElements));
//            minprox_dest = toolBox::doFindClosestProximityIt(pfrom, begin,
//                                                             acceptFilter,
//                                                             distance);

//            d++;// we look for boxed located at d+1

//            //take the first on that satisfy filters
//            if (minprox_dest != NULL) return minprox_dest;
//        }
//    }

//    return NULL;
//}

}

}

}


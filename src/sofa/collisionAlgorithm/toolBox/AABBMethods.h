#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/toolBox/ClosestProximityAlgorithm.h>

namespace sofa {

namespace collisionAlgorithm {

namespace toolBox {

//void fillElementSet(const BaseGeometry::BroadPhase::SPtr decorator, type::Vec3i cbox, std::set<BaseProximity::Index> & selectElements, int d) ;

BaseProximity::SPtr findClosestProximity(const BaseProximity::SPtr & pfrom, BaseGeometry *geo,
                                         std::function<bool(const collisionAlgorithm::PairDetection & )> acceptFilter = noFilter,
                                         std::function<double(const collisionAlgorithm::PairDetection & )> distance = distance3d) {
    return doFindClosestProximityIt(pfrom,geo->begin(),acceptFilter,distance);
}

}

} // namespace component

} // namespace sofa


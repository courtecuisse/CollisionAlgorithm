#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>

namespace sofa {

namespace collisionAlgorithm {

namespace toolBox {

static bool noFilter(const collisionAlgorithm::PairDetection & ) { return true; }

static double distance3d(const collisionAlgorithm::PairDetection & p ) { return (p.first->getPosition() - p.second->getPosition()).norm(); }

BaseProximity::SPtr doFindClosestProximityIt(BaseProximity::SPtr pfrom,
                                             BaseElementIterator::SPtr begin,
                                             std::function<bool(const collisionAlgorithm::PairDetection & )> acceptFilter = noFilter,
                                             std::function<double(const collisionAlgorithm::PairDetection & )> distance = distance3d);

}

} // namespace component

} // namespace sofa


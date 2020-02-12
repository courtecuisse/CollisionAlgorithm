#include <sofa/collisionAlgorithm/toolBox/ClosestProximityAlgorithm.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/collisionAlgorithm/geometry/SubsetGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

namespace toolBox {

BaseProximity::SPtr doFindClosestProximityIt(const BaseProximity::SPtr & pfrom, BaseElementIterator::UPtr & begin,
                                             std::function<bool(const collisionAlgorithm::PairDetection & )> acceptFilter,
                                             std::function<double(const collisionAlgorithm::PairDetection & )> distance) {

    double min_dist = std::numeric_limits<double>::max();
    BaseProximity::SPtr minprox_dest = nullptr;
    defaulttype::Vector3 P = pfrom->getPosition();

    while (! begin->end()) {
        BaseProximity::SPtr pdest = begin->project(P);

        collisionAlgorithm::PairDetection pair(pfrom,pdest);

        if (acceptFilter(pair)) {
            double dist = distance(pair);

            if (dist<min_dist) {
                min_dist = dist;
                minprox_dest = pdest;
            }
        }

        begin++;
    }

    return minprox_dest;
}

}

}

}


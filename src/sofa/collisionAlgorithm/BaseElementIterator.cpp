#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>

namespace sofa {

namespace collisionAlgorithm {

BaseElementIterator::BaseElementIterator(BaseGeometry * geo) {
    if (geo) geo->updateTime();
}

}

}

#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa::collisionAlgorithm {

PointElementSPtr::PointElementSPtr(BaseProximity::SPtr prox)
: std::shared_ptr<PointElement>(new PointElement(prox)) {
    get()->insertElement(*this);
}

BaseProximity::SPtr PointElementSPtr::createProximity() const {
    return get()->getP0();
}

}

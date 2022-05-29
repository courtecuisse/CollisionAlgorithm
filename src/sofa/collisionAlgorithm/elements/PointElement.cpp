#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa::collisionAlgorithm {

PointElementSPtr PointElement::create(BaseProximity::SPtr prox) {
    return PointElementSPtr(prox);
}

PointElementSPtr::PointElementSPtr(BaseProximity::SPtr prox)
: std::shared_ptr<PointElement>(new PointElement(prox)) {
    get()->_pointElements().insert(*this);
}


BaseProximity::SPtr PointElementSPtr::createProximity() const {
    return get()->getP0();
}

}

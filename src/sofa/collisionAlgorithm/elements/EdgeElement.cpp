#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa::collisionAlgorithm {

EdgeElementSPtr EdgeElement::create(PointElement::SPtr p0, PointElement::SPtr p1) {
    return EdgeElementSPtr(p0,p1);
}

EdgeElementSPtr EdgeElement::create(BaseProximity::SPtr p0,BaseProximity::SPtr p1) {
    return EdgeElementSPtr(PointElement::create(p0),
                           PointElement::create(p1));
}

EdgeElementSPtr::EdgeElementSPtr(PointElement::SPtr point0, PointElement::SPtr point1)
: std::shared_ptr<EdgeElement>(new EdgeElement()) {
    get()->_pointElements().insert(point0);
    get()->_pointElements().insert(point1);

    get()->_edgeElements().insert(*this);
}

BaseProximity::SPtr EdgeElementSPtr::createProximity(double f0,double f1) const {
    return EdgeProximity::SPtr(new EdgeProximity(*this,f0,f1));
}

}

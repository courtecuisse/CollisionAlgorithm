#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa::collisionAlgorithm {

EdgeElementSPtr::EdgeElementSPtr(BaseProximity::SPtr p0,BaseProximity::SPtr p1)
: EdgeElementSPtr(PointElement::SPtr(p0),PointElement::SPtr(p1)) {}

EdgeElementSPtr::EdgeElementSPtr(PointElement::SPtr point0, PointElement::SPtr point1)
: std::shared_ptr<EdgeElement>(new EdgeElement()) {
    get()->insertElement(point0);
    get()->insertElement(point1);

    get()->insertElement(*this);
}

BaseProximity::SPtr EdgeElementSPtr::createProximity(double f0,double f1) const {
    return EdgeProximity::SPtr(new EdgeProximity(*this,f0,f1));
}

}

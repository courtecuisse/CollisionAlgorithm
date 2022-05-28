#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa::collisionAlgorithm {

BaseProximity::SPtr EdgeElement::createProximity(double f0,double f1) const {
    return EdgeProximity::SPtr(new EdgeProximity(this->sptr(),f0,f1));
}

}

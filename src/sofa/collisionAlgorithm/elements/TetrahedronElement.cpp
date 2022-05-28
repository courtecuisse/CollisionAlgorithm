#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>

namespace sofa::collisionAlgorithm {

BaseProximity::SPtr TetrahedronElement::createProximity(double f0,double f1,double f2,double f3) const {
    return TetrahedronProximity::SPtr(new TetrahedronProximity(this->sptr(),f0,f1,f2,f3));
}

}

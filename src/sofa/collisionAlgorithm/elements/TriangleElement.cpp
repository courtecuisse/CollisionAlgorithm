#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa::collisionAlgorithm {

BaseProximity::SPtr TriangleElement::createProximity(double f0,double f1,double f2) const {
    return TriangleProximity::SPtr(new TriangleProximity(this->sptr(),f0,f1,f2));
}

}

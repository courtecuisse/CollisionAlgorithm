#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa::collisionAlgorithm {

PointElement::SPtr PointElement::create(const BaseProximity::SPtr &prox) {
    return PointElement::SPtr(new PointElement(prox));
}

}

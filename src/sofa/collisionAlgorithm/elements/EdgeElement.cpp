#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa::collisionAlgorithm {

EdgeElement::SPtr EdgeElement::create(PointElement::SPtr p0, PointElement::SPtr p1) {
    EdgeElement::SPtr res = std::shared_ptr<EdgeElement>(new EdgeElement());

    res->m_pointElements.insert(p0);
    res->m_pointElements.insert(p1);

//    res->m_edgeElements.insert(res.get());

    return res;
}

EdgeElement::SPtr EdgeElement::create(BaseProximity::SPtr p0,BaseProximity::SPtr p1) {
    return EdgeElement::create(PointElement::create(p0),
                               PointElement::create(p1));
}



}

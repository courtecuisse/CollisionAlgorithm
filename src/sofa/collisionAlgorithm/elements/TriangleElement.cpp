#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa::collisionAlgorithm {

TriangleElement::SPtr TriangleElement::create(BaseProximity::SPtr prox0, BaseProximity::SPtr prox1,BaseProximity::SPtr prox2) {
    PointElement::SPtr p0 = PointElement::create(prox0);
    PointElement::SPtr p1 = PointElement::create(prox1);
    PointElement::SPtr p2 = PointElement::create(prox2);

    EdgeElement::SPtr e0 = EdgeElement::create(p0,p1);
    EdgeElement::SPtr e1 = EdgeElement::create(p1,p2);
    EdgeElement::SPtr e2 = EdgeElement::create(p2,p0);

    return TriangleElement::create(e0,e1,e2);
}

TriangleElement::SPtr TriangleElement::create(EdgeElement::SPtr edge0, EdgeElement::SPtr edge1, EdgeElement::SPtr edge2) {
    auto res = std::shared_ptr<TriangleElement>(new TriangleElement());

    res->m_pointElements.insert(edge0->pointElements()[0]);
    res->m_pointElements.insert(edge0->pointElements()[1]);
    res->m_pointElements.insert(edge1->pointElements()[0]);
    res->m_pointElements.insert(edge1->pointElements()[1]);

    res->m_edgeElements.insert(edge0);
    res->m_edgeElements.insert(edge1);
    res->m_edgeElements.insert(edge2);

    res->m_pointElements[0]->triangleAround().insert(res);
    res->m_pointElements[1]->triangleAround().insert(res);
    res->m_pointElements[2]->triangleAround().insert(res);

//    res->m_triangleElements.insert(res.get());

    return res;
}



}

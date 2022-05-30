#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>

namespace sofa::collisionAlgorithm {

TetrahedronElement::SPtr TetrahedronElement::create(TriangleElement::SPtr tri0, TriangleElement::SPtr tri1, TriangleElement::SPtr tri2, TriangleElement::SPtr tri3) {
    TetrahedronElement::SPtr res = std::shared_ptr<TetrahedronElement>(new TetrahedronElement());

    res->m_pointElements.insert(tri0->pointElements()[0]);
    res->m_pointElements.insert(tri0->pointElements()[1]);
    res->m_pointElements.insert(tri0->pointElements()[2]);
    res->m_pointElements.insert(tri1->pointElements()[0]);
    res->m_pointElements.insert(tri1->pointElements()[1]);
    res->m_pointElements.insert(tri1->pointElements()[2]);

    res->m_edgeElements.insert(tri0->edgeElements()[0]);
    res->m_edgeElements.insert(tri0->edgeElements()[1]);
    res->m_edgeElements.insert(tri0->edgeElements()[2]);
    res->m_edgeElements.insert(tri1->edgeElements()[0]);
    res->m_edgeElements.insert(tri1->edgeElements()[1]);
    res->m_edgeElements.insert(tri1->edgeElements()[2]);

    res->m_triangleElements.insert(tri0);
    res->m_triangleElements.insert(tri1);
    res->m_triangleElements.insert(tri2);
    res->m_triangleElements.insert(tri3);

//    res->m_tetrahedronElements.insert(res.get());

    return res;
}

TetrahedronElement::SPtr TetrahedronElement::create(BaseProximity::SPtr prox0, BaseProximity::SPtr prox1,BaseProximity::SPtr prox2,BaseProximity::SPtr prox3) {
    PointElement::SPtr p0 = PointElement::create(prox0);
    PointElement::SPtr p1 = PointElement::create(prox1);
    PointElement::SPtr p2 = PointElement::create(prox2);
    PointElement::SPtr p3 = PointElement::create(prox3);

    EdgeElement::SPtr e0 = EdgeElement::create(p0,p1);
    EdgeElement::SPtr e1 = EdgeElement::create(p1,p2);
    EdgeElement::SPtr e2 = EdgeElement::create(p2,p0);

    EdgeElement::SPtr e3 = EdgeElement::create(p3,p0);
    EdgeElement::SPtr e4 = EdgeElement::create(p3,p1);
    EdgeElement::SPtr e5 = EdgeElement::create(p3,p2);

    TriangleElement::SPtr t0 = TriangleElement::create(e0,e1,e2);
    TriangleElement::SPtr t1 = TriangleElement::create(e0,e4,e3);
    TriangleElement::SPtr t2 = TriangleElement::create(e4,e1,e5);
    TriangleElement::SPtr t3 = TriangleElement::create(e3,e2,e5);

    return TetrahedronElement::create(t0,t1,t2,t3);
}

}

#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>

namespace sofa::collisionAlgorithm {

TetrahedronElementSPtr TetrahedronElement::create(TriangleElement::SPtr tri0, TriangleElement::SPtr tri1, TriangleElement::SPtr tri2, TriangleElement::SPtr tri3) {
    return TetrahedronElementSPtr(tri0,tri1,tri2,tri3);
}

TetrahedronElementSPtr TetrahedronElement::create(BaseProximity::SPtr prox0, BaseProximity::SPtr prox1,BaseProximity::SPtr prox2,BaseProximity::SPtr prox3) {
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

    return TetrahedronElementSPtr(TriangleElement::create(e0,e1,e2),
                                  TriangleElement::create(e0,e4,e3),
                                  TriangleElement::create(e4,e1,e5),
                                  TriangleElement::create(e3,e2,e5));
}

TetrahedronElementSPtr::TetrahedronElementSPtr(TriangleElement::SPtr tri0, TriangleElement::SPtr tri1, TriangleElement::SPtr tri2, TriangleElement::SPtr tri3)
: std::shared_ptr<TetrahedronElement>(new TetrahedronElement()) {

    get()->_pointElements().insert(tri0->pointElements()[0]);
    get()->_pointElements().insert(tri0->pointElements()[1]);
    get()->_pointElements().insert(tri0->pointElements()[2]);
    get()->_pointElements().insert(tri1->pointElements()[0]);
    get()->_pointElements().insert(tri1->pointElements()[1]);
    get()->_pointElements().insert(tri1->pointElements()[2]);

    get()->_edgeElements().insert(tri0->edgeElements()[0]);
    get()->_edgeElements().insert(tri0->edgeElements()[1]);
    get()->_edgeElements().insert(tri0->edgeElements()[2]);
    get()->_edgeElements().insert(tri1->edgeElements()[0]);
    get()->_edgeElements().insert(tri1->edgeElements()[1]);
    get()->_edgeElements().insert(tri1->edgeElements()[2]);

    get()->_triangleElements().insert(tri0);
    get()->_triangleElements().insert(tri1);
    get()->_triangleElements().insert(tri2);
    get()->_triangleElements().insert(tri3);

    get()->_tetrahedronElements().insert(*this);
}

BaseProximity::SPtr TetrahedronElementSPtr::createProximity(double f0,double f1,double f2,double f3) const {
    return TetrahedronProximity::SPtr(new TetrahedronProximity(*this,f0,f1,f2,f3));
}

}

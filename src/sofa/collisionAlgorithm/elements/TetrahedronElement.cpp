#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/TetrahedronProximity.h>

namespace sofa::collisionAlgorithm {

TetrahedronElementSPtr::TetrahedronElementSPtr(BaseProximity::SPtr p0, BaseProximity::SPtr p1,BaseProximity::SPtr p2,BaseProximity::SPtr p3)
: std::shared_ptr<TetrahedronElement>(new TetrahedronElement()) {
    get()->insertElement(PointElement::SPtr(p0));
    get()->insertElement(PointElement::SPtr(p1));
    get()->insertElement(PointElement::SPtr(p2));
    get()->insertElement(PointElement::SPtr(p3));

    get()->insertElement(EdgeElement::SPtr(p0,p1));
    get()->insertElement(EdgeElement::SPtr(p1,p2));
    get()->insertElement(EdgeElement::SPtr(p2,p0));

    get()->insertElement(EdgeElement::SPtr(p3,p0));
    get()->insertElement(EdgeElement::SPtr(p3,p1));
    get()->insertElement(EdgeElement::SPtr(p3,p2));

    get()->insertElement(TriangleElement::SPtr(p0,p1,p2));
    get()->insertElement(TriangleElement::SPtr(p1,p1,p3));
    get()->insertElement(TriangleElement::SPtr(p1,p2,p3));
    get()->insertElement(TriangleElement::SPtr(p2,p0,p3));

    get()->insertElement(*this);
}

TetrahedronElementSPtr::TetrahedronElementSPtr(TriangleElement::SPtr tri0, TriangleElement::SPtr tri1, TriangleElement::SPtr tri2, TriangleElement::SPtr tri3)
: std::shared_ptr<TetrahedronElement>(new TetrahedronElement()) {

    get()->insertElement(tri0->pointElements()[0]);
    get()->insertElement(tri0->pointElements()[1]);
    get()->insertElement(tri0->pointElements()[2]);
    get()->insertElement(tri1->pointElements()[0]);
    get()->insertElement(tri1->pointElements()[1]);
    get()->insertElement(tri1->pointElements()[2]);

    get()->insertElement(tri0->edgeElements()[0]);
    get()->insertElement(tri0->edgeElements()[1]);
    get()->insertElement(tri0->edgeElements()[2]);
    get()->insertElement(tri1->edgeElements()[0]);
    get()->insertElement(tri1->edgeElements()[1]);
    get()->insertElement(tri1->edgeElements()[2]);

    get()->insertElement(tri0);
    get()->insertElement(tri1);
    get()->insertElement(tri2);
    get()->insertElement(tri3);

    get()->insertElement(*this);
}

BaseProximity::SPtr TetrahedronElementSPtr::createProximity(double f0,double f1,double f2,double f3) const {
    return TetrahedronProximity::SPtr(new TetrahedronProximity(*this,f0,f1,f2,f3));
}

}

#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa::collisionAlgorithm {

TriangleElementSPtr::TriangleElementSPtr(EdgeElement::SPtr edge0, EdgeElement::SPtr edge1, EdgeElement::SPtr edge2)
: std::shared_ptr<TriangleElement>(new TriangleElement()) {
    get()->_pointElements().insert(edge0->pointElements()[0]);
    get()->_pointElements().insert(edge0->pointElements()[1]);
    get()->_pointElements().insert(edge1->pointElements()[0]);
    get()->_pointElements().insert(edge1->pointElements()[1]);

    get()->_edgeElements().insert(edge0);
    get()->_edgeElements().insert(edge1);
    get()->_edgeElements().insert(edge2);

    get()->_triangleElements().insert(*this);





    get()->_pointElements()[0]->triangleAround().insert(*this);
    get()->_pointElements()[1]->triangleAround().insert(*this);
    get()->_pointElements()[2]->triangleAround().insert(*this);

}

TriangleElementSPtr TriangleElement::create(BaseProximity::SPtr prox0, BaseProximity::SPtr prox1,BaseProximity::SPtr prox2) {
    PointElement::SPtr p0 = PointElement::create(prox0);
    PointElement::SPtr p1 = PointElement::create(prox1);
    PointElement::SPtr p2 = PointElement::create(prox2);

    return TriangleElementSPtr(EdgeElement::create(p0,p1),
                               EdgeElement::create(p1,p2),
                               EdgeElement::create(p2,p0));
}

TriangleElementSPtr TriangleElement::create(EdgeElement::SPtr edge0, EdgeElement::SPtr edge1, EdgeElement::SPtr edge2) {
    return TriangleElementSPtr(edge0,edge1,edge2);
}

BaseProximity::SPtr TriangleElementSPtr::createProximity(double f0,double f1,double f2) const {
    return TriangleProximity::SPtr(new TriangleProximity(*this,f0,f1,f2));
}

}

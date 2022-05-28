#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa::collisionAlgorithm {

TriangleElementSPtr::TriangleElementSPtr(BaseProximity::SPtr p0, BaseProximity::SPtr p1,BaseProximity::SPtr p2)
: std::shared_ptr<TriangleElement>(new TriangleElement()) {
    get()->insertElement(PointElement::SPtr(p0));
    get()->insertElement(PointElement::SPtr(p1));
    get()->insertElement(PointElement::SPtr(p2));

    get()->insertElement(EdgeElement::SPtr(p0,p1));
    get()->insertElement(EdgeElement::SPtr(p1,p2));
    get()->insertElement(EdgeElement::SPtr(p2,p0));

    get()->insertElement(*this);
}

TriangleElementSPtr::TriangleElementSPtr(EdgeElement::SPtr edge0, EdgeElement::SPtr edge1, EdgeElement::SPtr edge2)
: std::shared_ptr<TriangleElement>(new TriangleElement()) {
    get()->insertElement(edge0->pointElements()[0]);
    get()->insertElement(edge0->pointElements()[1]);
    get()->insertElement(edge1->pointElements()[0]);
    get()->insertElement(edge1->pointElements()[1]);

    get()->insertElement(edge0);
    get()->insertElement(edge1);
    get()->insertElement(edge2);

    get()->insertElement(*this);
}

BaseProximity::SPtr TriangleElementSPtr::createProximity(double f0,double f1,double f2) const {
    return TriangleProximity::SPtr(new TriangleProximity(*this,f0,f1,f2));
}

}

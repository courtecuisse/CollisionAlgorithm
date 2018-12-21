#include <sofa/collisionAlgorithm/addons/geometry/AABBDecorator.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(AABBDecorator)

int AABBDecoratorClass = core::RegisterObject("AABBDecorator")
.add< AABBDecorator >();

}

}

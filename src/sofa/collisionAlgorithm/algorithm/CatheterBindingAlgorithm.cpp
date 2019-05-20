#include <sofa/collisionAlgorithm/algorithm/CatheterBindingAlgorithm.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

SOFA_DECL_CLASS(CatheterBindingAlgorithm)

int CatheterBindingAlgorithmClass = core::RegisterObject("CatheterBindingAlgorithm")
.add< CatheterBindingAlgorithm >();

}

}




#include <sofa/collisionAlgorithm/filters/SubsetFilter.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa {

namespace collisionAlgorithm {

int SubsetFilterClass = core::RegisterObject("SubsetFilter")
.add< SubsetFilter >();
}

}
#include <sofa/collisionAlgorithm/algorithm/PointCloudBindingAlgorithm.inl>

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa {

namespace collisionAlgorithm {


SOFA_DECL_CLASS(PointCloudBindingAlgorithm)

int PointCloudBindingAlgorithmClass = core::RegisterObject("Point Cloud Binding Algorithm")
.add<PointCloudBindingAlgorithm >()
;

} // namespace collisionAlgorithm

} // namespace sofa

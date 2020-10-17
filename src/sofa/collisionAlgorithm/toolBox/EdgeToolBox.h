#pragma once


#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/SolidTypes.h>


namespace sofa {

namespace collisionAlgorithm {

typedef defaulttype::Vector3 Vec3d;

namespace toolBox {
//------------ Edges methods ------------//

void projectOnEdge(const Vec3d & projP, const Vec3d & e1, const Vec3d & e2, double & fact_u, double & fact_v);

}

} // namespace component

} // namespace sofa


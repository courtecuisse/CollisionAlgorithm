#include <sofa/collisionAlgorithm/toolBox/EdgeToolBox.h>


namespace sofa
{

namespace collisionAlgorithm
{


namespace toolBox
{

void projectOnEdge(const Vec3d & projP, const Vec3d & e1, const Vec3d & e2, double & fact_u, double & fact_v)
{

    Vec3d v = e2 - e1;
    fact_v = dot(projP - e1,v) / dot(v,v);

    if (fact_v<0.0) fact_v = 0.0;
    else if (fact_v>1.0) fact_v = 1.0;

    fact_u = 1.0-fact_v;
}



}

} // namespace component

} // namespace sofa


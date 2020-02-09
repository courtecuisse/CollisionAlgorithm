#pragma once

#include <sofa/collisionAlgorithm/toolBox/EdgeToolBox.h>


namespace sofa
{

namespace collisionAlgorithm
{

typedef defaulttype::Vector3 Vec3d;

struct TriangleInfo
{
    Vec3d v0,v1;
    double d00;
    double d01;
    double d11;
    double invDenom;

    Vec3d ax1,ax2;
    Vec3d P0,P1,P2;
};

namespace toolBox
{
//------------ Triangle methods ------------//

TriangleInfo computeTriangleInfo(const Vec3d & t0,const Vec3d & t1,const Vec3d & t2);

void computeTriangleBaryCoords(const Vec3d & proj_P, const TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w);

void projectOnTriangle(const Vec3d projectP, const TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w);


}

} // namespace component

} // namespace sofa


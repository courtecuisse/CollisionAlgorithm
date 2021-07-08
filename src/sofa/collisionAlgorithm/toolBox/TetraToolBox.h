#pragma once

#include <sofa/collisionAlgorithm/toolBox/TriangleToolBox.h>


namespace sofa
{

namespace collisionAlgorithm
{

typedef type::Vector3 Vec3d;


struct TetraInfo
{
    double V0;
    Vec3d P0,P1,P2,P3;
    Vec3d ax1,ax2,ax3,ax2Cax3;
};

namespace toolBox
{

//------------ Tetrahedra methods ------------//

TetraInfo computeTetraInfo(const Vec3d & p0, const Vec3d & p1, const Vec3d & p2, const Vec3d & p3);

void computeTetraBaryCoords(const Vec3d & P, const TetraInfo & tinfo, double & fact_u,double & fact_v, double & fact_w, double & fact_x);

void projectOnTetra(const Vec3d & projectP, const TetraInfo & tinfo, double & fact_u, double & fact_v, double & fact_w, double & fact_x);


}

} // namespace component

} // namespace sofa


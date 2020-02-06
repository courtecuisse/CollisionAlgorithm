#include <sofa/collisionAlgorithm/toolBox/TetraToolBox.h>


namespace sofa
{

namespace collisionAlgorithm
{


namespace toolBox
{


void computeTetraBaryCoords(const Vec3d & P, const TetraInfo & tinfo, double & fact_u,double & fact_v, double & fact_w, double & fact_x)
{
    Vec3d e = P - tinfo.P0;

    double Va = 1.0/6.0 * dot(e,tinfo.ax2Cax3);
    double Vb = 1.0/6.0 * dot(tinfo.ax1,e.cross(tinfo.ax3));
    double Vc = 1.0/6.0 * dot(tinfo.ax1,tinfo.ax2.cross(e));

    fact_v = Va/tinfo.V0;
    fact_w = Vb/tinfo.V0;
    fact_x = Vc/tinfo.V0;
    fact_u = 1.0 - (fact_v + fact_w + fact_x);

}

void projectOnTetra(const Vec3d & projectP, const TetraInfo & tinfo, double & fact_u, double & fact_v, double & fact_w, double & fact_x)
{
    toolBox::computeTetraBaryCoords(projectP, tinfo, fact_u,fact_v,fact_w,fact_x);
    if(fact_u<0)
    {
        fact_u = 0;
        projectOnTriangle(projectP, computeTriangleInfo(tinfo.P1, tinfo.P2, tinfo.P3), fact_v,fact_w,fact_x);
    }
    else if(fact_v<0)
    {
        fact_v = 0;
        projectOnTriangle(projectP, computeTriangleInfo(tinfo.P0, tinfo.P2, tinfo.P3),fact_u,fact_w,fact_x);
    }
    else if(fact_w<0)
    {
        fact_w = 0;
        projectOnTriangle(projectP, computeTriangleInfo(tinfo.P0, tinfo.P1, tinfo.P3),fact_u,fact_v,fact_x);
    }
    else if(fact_x<0)
    {
        fact_x = 0;
        projectOnTriangle(projectP, computeTriangleInfo(tinfo.P0, tinfo.P1, tinfo.P2),fact_u,fact_v,fact_w);
    }
}

TetraInfo computeTetraInfo(const Vec3d & p0, const Vec3d & p1, const Vec3d & p2, const Vec3d & p3)
{
    TetraInfo tinfo;

    tinfo.P0 = p0;
    tinfo.P1 = p1;
    tinfo.P2 = p2;
    tinfo.P3 = p3;

    tinfo.ax1 = p1 - p0;
    tinfo.ax2 = p2 - p0;
    tinfo.ax3 = p3 - p0;
    tinfo.ax2Cax3 = tinfo.ax2.cross(tinfo.ax3);
    tinfo.V0 = 1.0/6.0 * dot(tinfo.ax1,tinfo.ax2Cax3);

    return tinfo;
}


}

} // namespace component

} // namespace sofa


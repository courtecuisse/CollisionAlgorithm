#include <sofa/collisionAlgorithm/toolBox/TriangleToolBox.h>


namespace sofa
{

namespace collisionAlgorithm
{


namespace toolBox
{

TriangleInfo computeTriangleInfo(const Vec3d & t0,const Vec3d & t1,const Vec3d & t2) {
    //Compute the projection of the point on the plane
    TriangleInfo tinfo;

    tinfo.P0 = t0;
    tinfo.P1 = t1;
    tinfo.P2 = t2;

    tinfo.v0 = t1 - t0;
    tinfo.v1 = t2 - t0;
    Vec3d N=cross(tinfo.v0,tinfo.v1);
    N.normalize();

    tinfo.d00 = dot(tinfo.v0,tinfo.v0);
    tinfo.d01 = dot(tinfo.v0,tinfo.v1);
    tinfo.d11 = dot(tinfo.v1,tinfo.v1);

    tinfo.invDenom = 1.0 / (tinfo.d00 * tinfo.d11 - tinfo.d01 * tinfo.d01);

    tinfo.ax1 = tinfo.v0;
    tinfo.ax2 = tinfo.v0.cross(N);

    tinfo.ax1.normalize();
    tinfo.ax2.normalize();

    return tinfo;
}

void computeTriangleBaryCoords(const Vec3d & proj_P, const TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w)
{
    Vec3d v2 = proj_P - tinfo.P0;

    double d20 = dot(v2,tinfo.v0);
    double d21 = dot(v2,tinfo.v1);

    fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
    fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
    fact_u = 1.0 - fact_v  - fact_w;
}

void projectOnTriangle(const Vec3d projectP, const TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w)
{
    Vec3d x1x2 = projectP - tinfo.P0;

    //corrdinate on the plane
    double c0 = dot(x1x2,tinfo.ax1);
    double c1 = dot(x1x2,tinfo.ax2);
    Vec3d proj_P = tinfo.P0 + tinfo.ax1 * c0 + tinfo.ax2 * c1;

    computeTriangleBaryCoords(proj_P, tinfo, fact_u,fact_v,fact_w);

    if (fact_u<0)
    {
        projectOnEdge(proj_P, tinfo.P1, tinfo.P2, fact_v, fact_w);
        fact_u=0;
    }
    else if (fact_v<0)
    {
        projectOnEdge(proj_P, tinfo.P0, tinfo.P2, fact_u, fact_w);
        fact_v=0;
    }
    else if (fact_w<0)
    {
        projectOnEdge(proj_P, tinfo.P0, tinfo.P1, fact_u, fact_v);
        fact_w=0;
    }
}


}

} // namespace component

} // namespace sofa


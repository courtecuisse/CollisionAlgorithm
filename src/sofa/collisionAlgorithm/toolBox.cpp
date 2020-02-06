#include <sofa/collisionAlgorithm/toolBox.h>


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

void computeTriangleBaryCoords(const Vec3d & proj_P, const Vec3d & triangleP0, const TriangleInfo & tinfo, double & fact_u, double & fact_v, double & fact_w)
{
    Vec3d v2 = proj_P - triangleP0;

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

    computeTriangleBaryCoords(proj_P, tinfo.P0, tinfo, fact_u,fact_v,fact_w);

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


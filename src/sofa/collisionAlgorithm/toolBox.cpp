#include <sofa/collisionAlgorithm/toolBox.h>


namespace sofa
{

namespace collisionAlgorithm
{


namespace toolBox
{

inline void projectOnEdge(const Vec3d & projP, const Vec3d & e1, const Vec3d & e2, double & fact_u, double & fact_v)
{

    Vec3d v = e2 - e1;
    fact_v = dot(projP - e1,v) / dot(v,v);

    if (fact_v<0.0) fact_v = 0.0;
    else if (fact_v>1.0) fact_v = 1.0;

    fact_u = 1.0-fact_v;
}

inline TriangleInfo computeTriangleInfo(const Vec3d & t0,const Vec3d & t1,const Vec3d & t2) {
    //Compute the projection of the point on the plane
    TriangleInfo tinfo;
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

inline void computeTriangleBaryCoords(const Vec3d & proj_P,const TriangleInfo & tinfo, const Vec3d & p0, double & fact_u,double & fact_v, double & fact_w)
{
    Vec3d v2 = proj_P - p0;

    double d20 = dot(v2,tinfo.v0);
    double d21 = dot(v2,tinfo.v1);

    fact_v = (tinfo.d11 * d20 - tinfo.d01 * d21) * tinfo.invDenom;
    fact_w = (tinfo.d00 * d21 - tinfo.d01 * d20) * tinfo.invDenom;
    fact_u = 1.0 - fact_v  - fact_w;
}

inline void projectOnTriangle(const Vec3d projectP,
                              const Vec3d & triangleP0,
                              const Vec3d & triangleP1,
                              const Vec3d & triangleP2,
                              const TriangleInfo & tinfo,
                              double & fact_u,double & fact_v,double & fact_w)
{
    Vec3d x1x2 = projectP - triangleP0;

    //corrdinate on the plane
    double c0 = dot(x1x2,tinfo.ax1);
    double c1 = dot(x1x2,tinfo.ax2);
    Vec3d proj_P = triangleP0 + tinfo.ax1 * c0 + tinfo.ax2 * c1;

    computeTriangleBaryCoords(proj_P, tinfo, triangleP0, fact_u,fact_v,fact_w);

    if (fact_u<0)
    {
        Vec3d v3 = triangleP1 - triangleP2;
        Vec3d v4 = proj_P - triangleP2;
        double alpha = dot(v4,v3) / dot(v3,v3);

        if (alpha<0) alpha = 0;
        else if (alpha>1) alpha = 1;

        fact_u = 0;
        fact_v = alpha;
        fact_w = 1.0 - alpha;
    }
    else if (fact_v<0)
    {
        Vec3d v3 = triangleP0 - triangleP2;
        Vec3d v4 = proj_P - triangleP2;
        double alpha = dot(v4,v3) / dot(v3,v3);

        if (alpha<0) alpha = 0;
        else if (alpha>1) alpha = 1;

        fact_u = alpha;
        fact_v = 0;
        fact_w = 1.0 - alpha;
    }
    else if (fact_w<0)
    {
        Vec3d v3 = triangleP1 - triangleP0;
        Vec3d v4 = proj_P - triangleP0;
        double alpha = dot(v4,v3) / dot(v3,v3);

        if (alpha<0) alpha = 0;
        else if (alpha>1) alpha = 1;

        fact_u = 1.0 - alpha;
        fact_v = alpha;
        fact_w = 0;
    }
}


inline void computeTetraBaryCoords(const Vec3d & P,const TetraInfo & tinfo, double & fact_u,double & fact_v, double & fact_w, double & fact_x)
{
    Vec3d e = P - tinfo.p0;

    double Va = 1.0/6.0 * dot(e,tinfo.ax2Cax3);
    double Vb = 1.0/6.0 * dot(tinfo.ax1,e.cross(tinfo.ax3));
    double Vc = 1.0/6.0 * dot(tinfo.ax1,tinfo.ax2.cross(e));

    fact_v = Va/tinfo.V0;
    fact_w = Vb/tinfo.V0;
    fact_x = Vc/tinfo.V0;
    fact_u = 1.0 - (fact_v + fact_w + fact_x);

}

inline void projectOnTetra(const Vec3d & projectP, const TetraInfo & tinfo, double & fact_u, double & fact_v, double & fact_w, double & fact_x)
{
    toolBox::computeTetraBaryCoords(projectP, tinfo, fact_u,fact_v,fact_w,fact_x);
    if(fact_u<0)
    {
        fact_u = 0;
        //projectOnTriangle(projectP,);
    }
}

inline TetraInfo computeTetraInfo(const Vec3d & p0, const Vec3d & p1, const Vec3d & p2, const Vec3d & p3)
{
    TetraInfo tinfo;

    tinfo.ax1 = p1 - p0;
    tinfo.ax2 = p2 - p0;
    tinfo.ax3 = p3 - p0;
    tinfo.p0 = p0;
    tinfo.ax2Cax3 = tinfo.ax2.cross(tinfo.ax3);
    tinfo.V0 = 1.0/6.0 * dot(tinfo.ax1,tinfo.ax2Cax3);

    tinfo.trianglesInfo.push_back(computeTriangleInfo(p1,p2,p3));
    tinfo.trianglesInfo.push_back(computeTriangleInfo(p0,p2,p3));
    tinfo.trianglesInfo.push_back(computeTriangleInfo(p0,p1,p3));
    tinfo.trianglesInfo.push_back(computeTriangleInfo(p0,p1,p2));


    return tinfo;
}


}

} // namespace component

} // namespace sofa


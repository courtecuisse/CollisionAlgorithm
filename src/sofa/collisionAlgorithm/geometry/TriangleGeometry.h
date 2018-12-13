#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

class TriangleElement;

class TriangleGeometry : public EdgeGeometry
{
    friend class TriangleElement;
    friend class TriangleProximity;

public:
    SOFA_CLASS(TriangleGeometry,EdgeGeometry);

    TriangleGeometry()
        : EdgeGeometry()
        , d_phongInterpolation(initData(&d_phongInterpolation, true, "phongInterpolation", "Consider Phong Normals (normals from each point) instead of normal at each triangle"))
    {

    }
    virtual ~TriangleGeometry() override {}

    virtual void init() override;

    virtual void prepareDetection() override;

    ConstraintProximity::SPtr createProximity(const TriangleElement * elmt,double f1,double f2,double f3, bool phongNormals = true) const;

    typedef struct
    {
        defaulttype::Vector3 v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;

        defaulttype::Vector3 tn,ax1,ax2;
    } TriangleInfo;

   Data<bool> d_phongInterpolation;

protected:
    std::vector<TriangleInfo> m_triangle_info;
    std::vector<defaulttype::Vector3> m_pointNormal;
};


}

}

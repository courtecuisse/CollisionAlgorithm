#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa
{

namespace collisionAlgorithm
{

class TriangleElement;

class TriangleGeometry : public BaseGeometry
{
    friend class TriangleElement;
    friend class TriangleProximity;

public:
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef helper::vector<Triangle> VecTriangles;

    SOFA_CLASS(TriangleGeometry,BaseGeometry);

    TriangleGeometry()
        : BaseGeometry()
        , d_triangles(initData(&d_triangles, VecTriangles(), "triangles", "Vector of Triangles"))
        , d_phongInterpolation(initData(&d_phongInterpolation, true, "phongInterpolation", "Consider Phong Normals (normals from each point) instead of normal at each triangle"))
        , l_topology(initLink("topology", "Link to topology"))
    {
        if(!l_topology.get())
            l_topology.setPath("@.");
    }

    virtual ~TriangleGeometry() override {}

    virtual void init() override;

    virtual void prepareDetection() override;

    ConstraintProximity::SPtr createProximity(const TriangleElement * elmt,double f1,double f2,double f3, bool phongNormals = true) const;

    inline const VecTriangles& triangles() const { return d_triangles.getValue(); }

    typedef struct
    {
        defaulttype::Vector3 v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;

        defaulttype::Vector3 tn,ax1,ax2;
    } TriangleInfo;

    Data<VecTriangles> d_triangles;
    Data<bool> d_phongInterpolation;

protected:
    std::vector<TriangleInfo> m_triangle_info;
    std::vector<defaulttype::Vector3> m_pointNormal;

    core::objectmodel::SingleLink<TriangleGeometry,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;
};


}

}

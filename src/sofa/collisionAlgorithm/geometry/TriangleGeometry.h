#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>


namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class TriangleProximity;

template<class DataTypes>
class TriangleGeometry : public TBaseGeometry<DataTypes> {
    friend class TriangleProximity<DataTypes>;

public:
    typedef TBaseGeometry<DataTypes> Inherit;
    SOFA_CLASS(SOFA_TEMPLATE(TriangleGeometry,DataTypes),Inherit);

    typedef DataTypes TDataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;

    Data<VecTriangles> d_triangles;

    TriangleGeometry()
    : d_triangles(initData(&d_triangles, VecTriangles(), "triangles", "Vector of Triangles")) {}

    virtual ~TriangleGeometry() override {}

    virtual void init() override;

    virtual void prepareDetection() override;

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const;

    typedef struct
    {
        defaulttype::Vector3 v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;

        defaulttype::Vector3 ax1,ax2;
    } TriangleInfo;

    inline const TriangleInfo& triangleInfo(size_t index) const
    {
        return this->m_triangle_info[index];
    }    

    inline const VecTriangles& triangles() const
    {
        return d_triangles.getValue();
    }

protected:
    std::vector<TriangleInfo> m_triangle_info;
    helper::vector<defaulttype::Vector3> m_triangle_normals;

    void computeBaryCoords(const defaulttype::Vector3 & proj_P,const TriangleInfo & tinfo, const defaulttype::Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) const;

    virtual void projectLinear(unsigned eid, const defaulttype::Vector3 & P, core::topology::BaseMeshTopology::Triangle & triangle, defaulttype::Vector3 & factor) const;

};


}

}

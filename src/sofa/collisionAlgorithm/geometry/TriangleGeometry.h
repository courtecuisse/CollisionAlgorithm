#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>


namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class TriangleProximity;

template<class DataTypes>
class TriangleGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef TriangleGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;

    friend class TriangleProximity<GEOMETRY>;

    SOFA_CLASS(GEOMETRY,Inherit);

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

    inline defaulttype::Vector3 getNormal(const TriangleProximity<GEOMETRY> * prox) const {
        std::cout << "TRIANGLE NORMAL" << std::endl;

        return m_triangle_normals[prox->m_eid];
    }

    inline Coord getPosition(core::VecCoordId v, const TriangleProximity<GEOMETRY> * prox) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(v);
        return pos[prox->m_pid[0]] * prox->m_fact[0] +
               pos[prox->m_pid[1]] * prox->m_fact[1] +
               pos[prox->m_pid[2]] * prox->m_fact[2];
    }

    void computeBaryCoords(const defaulttype::Vector3 & proj_P,const TriangleInfo & tinfo, const defaulttype::Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) const;

    virtual void projectLinear(unsigned eid, const defaulttype::Vector3 & P, core::topology::BaseMeshTopology::Triangle & triangle, defaulttype::Vector3 & factor) const;

protected:
    std::vector<TriangleInfo> m_triangle_info;
    helper::vector<defaulttype::Vector3> m_triangle_normals;

};


}

}

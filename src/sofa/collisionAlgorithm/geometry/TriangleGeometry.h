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

    SOFA_CLASS(GEOMETRY,Inherit);

    DataElemnt<sofa::core::topology::BaseMeshTopology::Triangle> d_triangles;

    TriangleGeometry()
    : d_triangles(initData(&d_triangles, "triangles", "Vector of Triangles")){}

    virtual ~TriangleGeometry() override {}

    virtual void init() override;

    virtual void prepareDetection() override;

    virtual void draw(const core::visual::VisualParams * vparams) override;

    virtual BaseElementIterator::UPtr getElementIterator(unsigned eid = 0) const;

    typedef struct
    {
        defaulttype::Vector3 v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;

        defaulttype::Vector3 ax1,ax2;
    } TriangleInfo;

    //default implementation
    template<class DERIVED_GEOMETRY>
    inline Coord getPosition(const TriangleProximity<DERIVED_GEOMETRY> * prox, core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(v);
        return pos[prox->m_pid[0]] * prox->m_fact[0] +
               pos[prox->m_pid[1]] * prox->m_fact[1] +
               pos[prox->m_pid[2]] * prox->m_fact[2];
    }

    inline defaulttype::Vector3 getNormal(const TriangleProximity<GEOMETRY> * prox) const {
        return m_triangle_normals[prox->m_eid];
    }

    void computeBaryCoords(const defaulttype::Vector3 & proj_P,const TriangleInfo & tinfo, const defaulttype::Vector3 & p0, double & fact_u,double & fact_v, double & fact_w) const;

    virtual void project(unsigned eid, const defaulttype::Vector3 & P, core::topology::BaseMeshTopology::Triangle & triangle, defaulttype::Vector3 & factor) const;

protected:
    std::vector<TriangleInfo> m_triangle_info;
    helper::vector<defaulttype::Vector3> m_triangle_normals;

};


}

}

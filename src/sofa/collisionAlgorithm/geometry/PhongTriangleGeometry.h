#pragma once

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class PhongTriangleProximity;

template<class DataTypes>
class PhongTriangleGeometry : public TriangleGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TriangleGeometry<DataTypes> Inherit;
    typedef PhongTriangleGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::Coord Coord;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    friend class PhongTriangleProximity<GEOMETRY>;

    SOFA_CLASS(GEOMETRY,Inherit);

    virtual ~PhongTriangleGeometry() override {}

    virtual BaseElementIterator::UPtr begin(unsigned eid) const;

    virtual void init() override;

    virtual void prepareDetection() override;

    inline const std::vector<defaulttype::Vector3>& pointNormals() const
    {
        return this->m_point_normals;
    }

    inline Coord getPosition(core::VecCoordId v, const TriangleProximity<GEOMETRY> * prox) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(v);
        return pos[prox->m_pid[0]] * prox->m_fact[0] +
               pos[prox->m_pid[1]] * prox->m_fact[1] +
               pos[prox->m_pid[2]] * prox->m_fact[2];
    }


    inline defaulttype::Vector3 getNormal(const TriangleProximity<GEOMETRY> * prox) const {
        std::cout << "PHONG NORMAL" << std::endl;

        return m_point_normals[prox->m_pid[0]] * prox->m_fact[0] +
               m_point_normals[prox->m_pid[1]] * prox->m_fact[1] +
               m_point_normals[prox->m_pid[2]] * prox->m_fact[2];
    }

protected:
    std::vector<defaulttype::Vector3> m_point_normals;
    std::vector< std::vector<TriangleID> > m_trianglesAroundVertex;

};


}

}

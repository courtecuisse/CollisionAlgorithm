#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

template<class GEOMETRY>
class EdgeProximity;

template<class DataTypes>
class EdgeGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef EdgeGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
    typedef helper::vector<Edge> VecEdges;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<VecEdges> d_edges;

    EdgeGeometry()
    : d_edges(initData(&d_edges, VecEdges(), "edges", "Vector of Edges")) {}

    virtual BaseElementIterator::UPtr getElementIterator(unsigned eid = 0) const;

    //default implementation
    template<class DERIVED_GEOMETRY>
    inline Coord getPosition(const EdgeProximity<DERIVED_GEOMETRY> * prox, core::VecCoordId v = core::VecCoordId::position()) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(v);
        return pos[prox->m_pid[0]] * prox->m_fact[0] + pos[prox->m_pid[1]] * prox->m_fact[1];
    }

    inline defaulttype::Vector3 getNormal(const EdgeProximity<GEOMETRY> * /*prox*/) const {
        return defaulttype::Vector3(1,0,0);
    }

    virtual void project(unsigned eid, const defaulttype::Vector3 & P, core::topology::BaseMeshTopology::Edge & edge, defaulttype::Vector2 & factor) const;

};

}

}

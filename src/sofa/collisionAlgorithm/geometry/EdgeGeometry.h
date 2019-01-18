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

    friend class EdgeProximity<GEOMETRY>;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<VecEdges> d_edges;

    EdgeGeometry()
    : d_edges(initData(&d_edges, VecEdges(), "edges", "Vector of Edges")) {}

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const;

protected:
    inline defaulttype::Vector3 getNormal(unsigned /*pid0*/, unsigned /*pid1*/, double /*fact0*/, double /*fact1*/) const {
        return defaulttype::Vector3(1,0,0);
    }

    inline Coord getPosition(core::VecCoordId v, unsigned pid0, unsigned pid1, double fact0, double fact1) const {
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(v);
        return pos[pid0] * fact0 + pos[pid1] * fact1;
    }

    void projectLinear(unsigned eid, const defaulttype::Vector3 & P, core::topology::BaseMeshTopology::Edge & edge, defaulttype::Vector2 & factor) const;
};

}

}

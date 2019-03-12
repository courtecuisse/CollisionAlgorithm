#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class CONTAINER>
class EdgeElement : public BaseElement {
public:
    typedef CONTAINER TContainer;
    typedef typename CONTAINER::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    EdgeElement(unsigned id,const CONTAINER * geo)
    : m_eid(id), m_container(geo) {}

    inline BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {
        sofa::core::topology::BaseMeshTopology::Edge edge = m_container->getEdge(m_eid);

        const helper::ReadAccessor<Data <VecCoord> >& x = m_container->getState()->read(core::VecCoordId::position());

        defaulttype::Vector2 factor = project(P, x[edge[0]], x[edge[1]]);

        return BaseProximity::create<EdgeProximity<DataTypes> >(m_container->getState(), edge[0],edge[1],factor[0],factor[1]);
    }

    inline BaseProximity::SPtr center() const {
        sofa::core::topology::BaseMeshTopology::Edge edge = m_container->getEdge(m_eid);

        return BaseProximity::create<EdgeProximity<DataTypes> >(m_container->getState(), edge[0],edge[1],0.5,0.5);
    }

    inline defaulttype::BoundingBox getBBox() const {
        sofa::core::topology::BaseMeshTopology::Edge edge = m_container->getEdge(m_eid);

        const helper::ReadAccessor<Data <VecCoord> >& x = m_container->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[edge[0]]);
        bbox.include(x[edge[1]]);
        return bbox;
    }

    static defaulttype::Vector2 project(const defaulttype::Vector3 & P,const defaulttype::Vector3 & E1,const defaulttype::Vector3 & E2) {
        double fact_u;
        double fact_v;

        defaulttype::Vector3 v = E2 - E1;
        fact_v = dot (P - E1,v) / dot(v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;

        return defaulttype::Vector2(fact_u,fact_v);
    }

protected:
    unsigned m_eid;
    const CONTAINER * m_container;
};

}

}

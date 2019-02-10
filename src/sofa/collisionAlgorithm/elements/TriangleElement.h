#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class TriangleElement : public BaseElement {
public:
    typedef GEOMETRY TGeometry;
    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    TriangleElement(unsigned id,const GEOMETRY * geo) : m_tid(id), m_geo(geo) {}

    inline BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {
        core::topology::BaseMeshTopology::Triangle triangle;
        defaulttype::Vector3 factor;
        m_geo->project(m_tid, P, triangle, factor);

        return BaseProximity::create<TriangleProximity<GEOMETRY> >(m_geo, m_tid,triangle[0],triangle[1],triangle[2],factor[0],factor[1],factor[2]);
    }

    inline BaseProximity::SPtr center() const {
        const core::topology::BaseMeshTopology::Triangle & triangle = m_geo->getTriangles()[m_tid];
        return BaseProximity::create<TriangleProximity<GEOMETRY> >(m_geo, m_tid,triangle[0],triangle[1],triangle[2],0.3333,0.3333,0.3333);
    }

    inline defaulttype::BoundingBox getBBox() const {
        const core::topology::BaseMeshTopology::Triangle & triangle = m_geo->getTriangles()[m_tid];
        const helper::ReadAccessor<Data <VecCoord> >& x = m_geo->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[triangle[0]]);
        bbox.include(x[triangle[1]]);
        bbox.include(x[triangle[2]]);
        return bbox;
    }

protected:
    unsigned m_tid;
    const GEOMETRY * m_geo;
};

}

}

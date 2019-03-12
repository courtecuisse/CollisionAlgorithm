#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class CONTAINER>
class PointElement : public BaseElement {
public:
    typedef CONTAINER TContainer;
    typedef typename CONTAINER::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    PointElement(unsigned id,const CONTAINER * geo) : m_id(id), m_container(geo) {}

    inline BaseProximity::SPtr project(const defaulttype::Vector3 & /*P*/) const {
        return BaseProximity::create<PointProximity<DataTypes> >(m_container->getState(),m_id);
    }

    inline BaseProximity::SPtr center() const {
        return BaseProximity::create<PointProximity<DataTypes> >(m_container->getState(),m_id);
    }

    inline defaulttype::BoundingBox getBBox() const {
        const helper::ReadAccessor<Data <VecCoord> >& x = m_container->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[m_id]);
        return bbox;
    }

protected:
    unsigned m_id;
    const CONTAINER * m_container;
};

}

}

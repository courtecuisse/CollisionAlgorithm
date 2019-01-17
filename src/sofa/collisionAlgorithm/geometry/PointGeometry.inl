#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
BaseElementIterator::UPtr PointGeometry<DataTypes>::begin(unsigned eid) const {
    return BaseElementIterator::UPtr(new DefaultElementIterator(eid,this->l_state->getSize(),this));
}

template<class DataTypes>
BaseProximity::SPtr PointGeometry<DataTypes>::project(unsigned pid, const defaulttype::Vector3 & /*P*/) const {
    return BaseProximity::SPtr(new PointProximity<DataTypes>(pid,this->l_state.get()));
}

template<class DataTypes>
BaseProximity::SPtr PointGeometry<DataTypes>::center(unsigned pid) const {
    return BaseProximity::SPtr(new PointProximity<DataTypes>(pid,this->l_state.get()));
}

template<class DataTypes>
defaulttype::BoundingBox PointGeometry<DataTypes>::getBBox(unsigned pid) const {
    const helper::ReadAccessor<DataVecCoord>& x = *this->l_state->read(core::VecCoordId::position());
    defaulttype::BoundingBox bbox;
    bbox.include(x[pid]);
    return bbox;
}


template<class DataTypes>
void PointGeometry<DataTypes>::draw(const core::visual::VisualParams *vparams) {
    if (! vparams->displayFlags().getShowCollisionModels())
        return;

    if (this->d_color.getValue()[3] == 0.0)
        return;

    glDisable(GL_LIGHTING);

//    for(ElementIterator it=elementIterator();!it.end();it.next()) {
//        m_elements[i]->draw(vparams);
//    }
}

}

}

#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/element/PointElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
ElementIterator::UPtr PointGeometry<DataTypes>::begin() const {
    return ElementIterator::UPtr(new PointElementIterator<DataTypes>(this));
}

template<class DataTypes>
ElementIterator::End PointGeometry<DataTypes>::end() const {
    return this->l_state->getSize();
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

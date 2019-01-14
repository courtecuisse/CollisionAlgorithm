#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/iterator/PointElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
BaseElement::Iterator PointGeometry<DataTypes>::begin(unsigned eid) const {
    return DefaultElement::Iterator(eid, new PointElementIterator<DataTypes>(this));
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

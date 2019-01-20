#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
void EdgeGeometry<DataTypes>::prepareDetection()
{
    d_edges.prepareDetection();
}


template<class DataTypes>
void EdgeGeometry<DataTypes>::init()
{

}

template<class DataTypes>
void EdgeGeometry<DataTypes>::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels())
        return;

    if (this->d_color.getValue()[3] == 0.0)
        return;

    glDisable(GL_LIGHTING);


}

}

}

#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/container/DataPointContainer.h>
namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class PointGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef DataTypes TDataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    SOFA_CLASS(GEOMETRY,Inherit);

    DataPointContainer<GEOMETRY> d_points;

    PointGeometry()
    : d_points(initData(&d_points, "points", "Vector of Positions")){}

    void draw(const core::visual::VisualParams *vparams) {
        if (! vparams->displayFlags().getShowCollisionModels())
            return;

        if (this->d_color.getValue()[3] == 0.0)
            return;

        glDisable(GL_LIGHTING);

    //    for(ElementIterator it=elementIterator();!it.end();it.next()) {
    //        m_elements[i]->draw(vparams);
    //    }
    }

};

}

}

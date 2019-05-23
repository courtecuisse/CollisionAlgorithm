#pragma once

#include <sofa/core/collision/Pipeline.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

/*!
 * \brief The BroadPhase class defines an interface for the collision detection broad phase
 */
class BaseBroadPhase : public BaseGeometry::BroadPhase
{
public:
    SOFA_ABSTRACT_CLASS(BaseBroadPhase, BaseGeometry::BroadPhase);

    Data<defaulttype::Vector4> d_color;
    core::objectmodel::SingleLink<BaseBroadPhase,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_geometry;

    /*!
     * \brief BroadPhase Constructor
     */
    BaseBroadPhase()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , l_geometry(initLink("geometry", "link to the geometry")) {}

    void init( ) override {
        if (l_geometry != NULL) {
            sout << "Register to geometry " << l_geometry->getName() << sendl;
            //l_geometry->addBroadPhase(this);
        } else serr << "No geometry found" << sendl;
    }
};


}

}

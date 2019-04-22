#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/core/BehaviorModel.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <qopengl.h>

namespace sofa {

namespace collisionAlgorithm {

class BroadPhase;

/*!
 * \brief The BaseGeometry class is an abstract class defining a basic geometry
 * iterates through Proximity elements and draws them
 */
template<class DataTypes>
class BaseGeometry : public core::objectmodel::BaseObject
{
public:

    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    SOFA_ABSTRACT_CLASS(SOFA_TEMPLATE(BaseGeometry,DataTypes),core::objectmodel::BaseObject);

    Data<defaulttype::Vector4> d_color;
    Data<double> d_drawScaleNormal;

    core::objectmodel::SingleLink<BaseGeometry<DataTypes>,State,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_state;

    BaseGeometry()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , d_drawScaleNormal(initData(&d_drawScaleNormal, 1.0, "drawScaleNormal", "Color of the collision model"))
    , l_state(initLink("mstate", "link to state")) {
        l_state.setPath("@.");
    }

    double getTime() const {
        return this->getContext()->getTime();
    }

    sofa::core::behavior::MechanicalState<DataTypes> * getState() const {
        return l_state.get();
    }
};

}

}


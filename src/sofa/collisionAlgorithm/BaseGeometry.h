#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/BehaviorModel.h>
#include <sofa/core/collision/Pipeline.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <memory>
#include <map>
#include <vector>
#include <qopengl.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BroadPhase;

class BaseGeometry : public core::collision::Pipeline
{
public:
    SOFA_ABSTRACT_CLASS(BaseGeometry,core::collision::Pipeline);

    Data<defaulttype::Vector4> d_color;

    BaseGeometry()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    {
        m_decorator = NULL;
    }

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const = 0;

    virtual const BaseGeometry * end() const {
        return this;
    }

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    void bwdInit( ) override {
        prepareDetection();
    }

    virtual void reset() override {}

    virtual void computeCollisionReset() override;

    virtual void computeCollisionResponse() override {}

    void computeCollisionDetection() override {}

    BroadPhase * getBroadPhase() const {
        return m_decorator;
    }

    void setDecorator(BroadPhase * d) {
        m_decorator = d;
    }

    void unsetDecorator(BroadPhase * d) {
        if (m_decorator == d) m_decorator = NULL;
    }

protected:
    virtual void doCollisionReset() override {}

    virtual void doCollisionDetection(const sofa::helper::vector<core::CollisionModel*>& /*collisionModels*/) override {}

    virtual void doCollisionResponse() override {}

    virtual std::set< std::string > getResponseList() const override {
        std::set< std::string > res;
        return res;
    }

    virtual void prepareDetection() {}

    BroadPhase * m_decorator;
};


template<class DataTypes>
class TBaseGeometry : public BaseGeometry
{
public:
    SOFA_ABSTRACT_CLASS(SOFA_TEMPLATE(TBaseGeometry,DataTypes),BaseGeometry);

    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    TBaseGeometry()
    : BaseGeometry()
    , l_state(initLink("mstate", "link to state")) {
        l_state.setPath("@.");
    }

    sofa::core::behavior::BaseMechanicalState * getState() const override {
        return l_state.get();
    }

    core::objectmodel::SingleLink<TBaseGeometry<DataTypes>,State,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_state;
};

}

}

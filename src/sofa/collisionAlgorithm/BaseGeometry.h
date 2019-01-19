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
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model")){}

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    void bwdInit( ) override {
        computeCollisionReset();
    }

    virtual BaseElementIterator::UPtr getElementIterator(unsigned eid = 0) const = 0;

    virtual void reset() override {}

    virtual void computeCollisionReset() override;

    virtual void computeCollisionResponse() override {}

    void computeCollisionDetection() override {}


protected:
    virtual void doCollisionReset() override {}

    virtual void doCollisionDetection(const sofa::helper::vector<core::CollisionModel*>& /*collisionModels*/) override {}

    virtual void doCollisionResponse() override {}

    virtual std::set< std::string > getResponseList() const override {
        std::set< std::string > res;
        return res;
    }

    virtual void prepareDetection() {}

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

    sofa::core::behavior::MechanicalState<DataTypes> * getState() const {
        return l_state.get();
    }

    core::objectmodel::SingleLink<TBaseGeometry<DataTypes>,State,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_state;
};

class DataElementIterator {
public:

    DataElementIterator(const BaseGeometry * geo = NULL)
        : m_geometry(geo)
        , m_broadPhase(NULL) {}

    friend std::ostream& operator<<(std::ostream& i, const DataElementIterator& /*t*/)  {
        return i;
    }

    friend std::istream& operator>>(std::istream& i, DataElementIterator& /*t*/) {
        return i;
    }

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const {
        return m_geometry->getElementIterator(eid);
    }

    virtual const BaseGeometry * end() const {
        return m_geometry;
    }

    void setBroadPhase(BroadPhase * d) const {
        m_broadPhase = d;
    }

    void unsetBroadPhase(BroadPhase * d) const {
        if (m_broadPhase == d) m_broadPhase = NULL;
    }

    const BroadPhase * getBroadPhase() const {
        return m_broadPhase;
    }

private:
    const BaseGeometry * m_geometry;
    mutable const BroadPhase * m_broadPhase;
};

}

}

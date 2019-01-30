#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/core/collision/Pipeline.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/visual/VisualParams.h>
#include <qopengl.h>

namespace sofa {

namespace collisionAlgorithm {

class BroadPhase;

class BaseGeometry : public core::collision::Pipeline
{
public:
    SOFA_ABSTRACT_CLASS(BaseGeometry,core::collision::Pipeline);

    Data<defaulttype::Vector4> d_color;
    Data<double> d_drawScaleNormal;

    BaseGeometry()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , d_drawScaleNormal(initData(&d_drawScaleNormal, 1.0, "drawScaleNormal", "Color of the collision model")){}

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const = 0;

    virtual const BaseGeometry * end() const {
        return this;
    }

    void setBroadPhase(BroadPhase * d) {
        m_broadPhase = d;
    }

    void unsetBroadPhase(BroadPhase * d) {
        if (m_broadPhase == d) m_broadPhase = NULL;
    }

    BroadPhase * getBroadPhase() const {
        return m_broadPhase;
    }

    virtual void prepareDetection() {}

    virtual void draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowNormals())
            return;

        if (this->d_color.getValue()[3] == 0.0)
            return;

        if (d_drawScaleNormal.getValue() == 0) return;

        for (auto it=begin();it!=end();it++) {
            BaseProximity::SPtr center = (*it)->center();
            vparams->drawTool()->drawArrow(center->getPosition(), center->getPosition() + center->getNormal() * d_drawScaleNormal.getValue(), d_drawScaleNormal.getValue() * 0.1, d_color.getValue());
        }
    }

private:
    virtual void reset() override {}

    virtual void doCollisionReset() override {}

    virtual void doCollisionDetection(const sofa::helper::vector<core::CollisionModel*>& /*collisionModels*/) override {}

    virtual void doCollisionResponse() override {}

    virtual std::set< std::string > getResponseList() const override {
        std::set< std::string > res;
        return res;
    }

    void computeCollisionDetection() {}

    void computeCollisionReset() {
        prepareDetection();
    }

    void computeCollisionResponse()  {}

    void bwdInit() {
        prepareDetection();
    }

    BroadPhase * m_broadPhase;
};


template<class DataTypes>
class TBaseGeometry : public BaseGeometry
{
public:
    typedef TBaseGeometry<DataTypes> GEOMETRY;
    SOFA_ABSTRACT_CLASS(GEOMETRY,BaseGeometry);

    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    core::objectmodel::SingleLink<TBaseGeometry<DataTypes>,State,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_state;

    TBaseGeometry()
    : BaseGeometry()
    , l_state(initLink("mstate", "link to state")) {
        l_state.setPath("@.");
    }

    sofa::core::behavior::MechanicalState<DataTypes> * getState() const {
        return l_state.get();
    }
};

}

}


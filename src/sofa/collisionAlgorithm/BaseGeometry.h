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
class BaseGeometry : public core::objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(BaseGeometry,core::objectmodel::BaseObject);

    Data<defaulttype::Vector4> d_color;
    Data<double> d_drawScaleNormal;

    BaseGeometry()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , d_drawScaleNormal(initData(&d_drawScaleNormal, 1.0, "drawScaleNormal", "Color of the collision model")){
        m_updateTime = -1.0; // for update first time
    }

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    inline BaseElementIterator::UPtr begin(unsigned eid = 0) {
        update(this->getContext()->getTime());
        return getElementIterator(eid);
    }

    void update(double time) {
        if (m_updateTime < time) {
            m_updateTime = time;
            prepareDetection();
        }
    }

    inline const BaseGeometry * end() {
        return this;
    }

    /// broadphase accessors
    inline void setBroadPhase(BroadPhase * d) {
        m_broadPhase = d;
    }

    BroadPhase * getBroadPhase() const { return m_broadPhase; }

    virtual void prepareDetection() {}

    virtual void draw(const core::visual::VisualParams * vparams) {
        if (! vparams->displayFlags().getShowNormals()) return;
        if (this->d_color.getValue()[3] == 0.0) return;
        if (d_drawScaleNormal.getValue() == 0) return;

        for (auto it=begin();it!=end();it++) {
            BaseProximity::SPtr center = (*it)->center();
            vparams->drawTool()->drawArrow(
                center->getPosition(),
                center->getPosition() + center->getNormal() * d_drawScaleNormal.getValue(),
                d_drawScaleNormal.getValue() * 0.1,
                d_color.getValue()
            );
        }
    }

    virtual BaseElementIterator::UPtr getElementIterator(unsigned eid = 0) const = 0;

protected:

    double m_updateTime;
    BroadPhase * m_broadPhase;
};

/*!
 * \class TBaseGeometry
 * \brief Template implementation of BaseGeometry
 */
template<class DataTypes>
class TBaseGeometry : public BaseGeometry
{
public:
    typedef TBaseGeometry<DataTypes> GEOMETRY;
    SOFA_ABSTRACT_CLASS(GEOMETRY,BaseGeometry);

    typedef DataTypes TDataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    core::objectmodel::SingleLink<TBaseGeometry<DataTypes>,State,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_state;

    TBaseGeometry()
    : l_state(initLink("mstate", "link to state")) {
        l_state.setPath("@.");
        m_broadPhase = NULL ;
    }

    sofa::core::behavior::MechanicalState<DataTypes> * getState() const {
        return l_state.get();
    }
};

}

}


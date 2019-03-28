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
class BaseGeometry : public core::BehaviorModel
{
public:
    SOFA_ABSTRACT_CLASS(BaseGeometry,core::BehaviorModel);

    Data<defaulttype::Vector4> d_color;
    Data<double> d_drawScaleNormal;
//    sofa::core::objectmodel::_datacallback_::DataCallback c_update;

    BaseGeometry()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , d_drawScaleNormal(initData(&d_drawScaleNormal, 1.0, "drawScaleNormal", "Color of the collision model")){
//        c_update.addCallback(std::bind(&BaseGeometry::prepareDetection,this));
    }

//    void init() {
//        core::objectmodel::BaseData * data = this->getState()->findData("position");
//        if (data) c_update.addInput(data);
//    }

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const = 0;

    virtual const BaseGeometry * end() const {
        return this;
    }

    /// broadphase accessors
    virtual void setBroadPhase(BroadPhase * d) = 0;
    virtual BroadPhase * getBroadPhase() const = 0;

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

protected:
    /// Computation of a new simulation step.
    virtual void updatePosition(SReal ) {
        prepareDetection();
    }
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
    }

    sofa::core::behavior::MechanicalState<DataTypes> * getState() const {
        return l_state.get();
    }

    void setBroadPhase(BroadPhase * d) {
        m_broadPhase = d;
    }

    BroadPhase * getBroadPhase() const {
        return m_broadPhase;
    }

protected:
    BroadPhase * m_broadPhase;
};

/*
template<class LINK>
class TLinkGeometry : public BaseGeometry
{
public:
    typedef TLinkGeometry<LINK> GEOMETRY;
    SOFA_ABSTRACT_CLASS(GEOMETRY,BaseGeometry);

    typedef typename LINK::TDataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    core::objectmodel::SingleLink<TLinkGeometry<LINK>,LINK,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    TLinkGeometry()
    : l_geometry(initLink("geometry", "link to state")) {
        l_geometry.setPath("@.");
    }

    sofa::core::behavior::MechanicalState<DataTypes> * getState() const {
        return l_geometry->getState();
    }

    void setBroadPhase(BroadPhase * d) {
        l_geometry->setBroadPhase(d);
    }

    BroadPhase * getBroadPhase() const {
        return l_geometry->getBroadPhase();
    }
};
*/

}

}


#pragma once

#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
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
    , d_drawScaleNormal(initData(&d_drawScaleNormal, 1.0, "drawScaleNormal", "Color of the collision model"))
    , m_broadPhase(NULL)
    , m_update_time(-1.0) {}

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) = 0;

    virtual unsigned end() const = 0;

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    void setBroadPhase(BroadPhase * d) {
        m_broadPhase = d;
    }

    void unsetBroadPhase(BroadPhase * d) {
        if (m_broadPhase == d) m_broadPhase = NULL;
    }

    const BroadPhase * getBroadPhase();

    virtual void prepareDetection() {}

    inline void updateTime() {
        double time = this->getContext()->getTime();

        if (m_update_time < 0) init();

        if (m_update_time < time) {
            m_update_time = time;
            prepareDetection();
        }
    }

protected:
    BroadPhase * m_broadPhase;
    double m_update_time;
};


template<class DataTypes, class PROXIMITYDATA>
class TBaseGeometry : public BaseGeometry {
public:

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Deriv Deriv1;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    SOFA_ABSTRACT_CLASS(SOFA_TEMPLATE2(TBaseGeometry,DataTypes,PROXIMITYDATA),BaseGeometry);

    core::objectmodel::SingleLink<TBaseGeometry<DataTypes,PROXIMITYDATA>,State,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_state;

    TBaseGeometry()
    : l_state(initLink("mstate", "link to state")) {
        l_state.setPath("@.");
    }

    inline void drawNormals(const core::visual::VisualParams *vparams) {
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

    inline sofa::core::behavior::MechanicalState<DataTypes> * getState() const {
        return l_state.get();
    }

    /*
    virtual defaulttype::BoundingBox getBBox(unsigned eid) const  = 0;

    virtual PROXIMITYDATA center(unsigned eid) const = 0;

    virtual PROXIMITYDATA project(unsigned eid,const defaulttype::Vector3 & P) const = 0;

    virtual defaulttype::Vector3 getPosition(const PROXIMITYDATA & data, core::VecCoordId v) const = 0;

    virtual defaulttype::Vector3 getNormal(const PROXIMITYDATA & data) const = 0;

    virtual void addContributions(const PROXIMITYDATA & data, MatrixDerivRowIterator & it, const defaulttype::Vector3 & N) const = 0;*/

    inline void buildJacobianConstraint(const PROXIMITYDATA & data, core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & normals, double fact, unsigned constraintId) const {
        DataMatrixDeriv & c1_d = *cId[this->getState()].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (unsigned j=0;j<normals.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);
            data.addContributions(c_it, normals[j] * fact);
        }

        c1_d.endEdit();
    }

    inline void storeLambda(const PROXIMITYDATA & /*data*/, const core::ConstraintParams* cParams, core::MultiVecDerivId resId, unsigned cid, const sofa::defaulttype::BaseVector* lambda) const {
        auto res = sofa::helper::write(*resId[this->getState()].write(), cParams);
        const typename DataTypes::MatrixDeriv& j = cParams->readJ(this->getState())->getValue();
        auto rowIt = j.readLine(cid);
        const double f = lambda->element(cid);
        for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt)
        {
            res[colIt.index()] += colIt.val() * f;
        }
    }


};

}

}


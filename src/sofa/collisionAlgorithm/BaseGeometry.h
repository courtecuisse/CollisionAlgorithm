#pragma once

#include <sofa/simulation/CollisionBeginEvent.h>
#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/core/BehaviorModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/simulation/Visitor.h>
#include <sofa/helper/system/gl.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa {

namespace collisionAlgorithm {

/*!
 * \brief The BaseGeometry class is an abstract class defining a basic geometry
 * iterates through Proximity elements and draws them
 */
class BaseGeometry : public core::objectmodel::BaseObject
{
public:

    class BroadPhase : public core::objectmodel::BaseObject {
    public:

        SOFA_ABSTRACT_CLASS(BroadPhase,core::objectmodel::BaseObject);

        /*!
         * \brief prepareDetection virtual method to implement
         * detection pre-processing
         */
        virtual void prepareDetection() = 0;

        /*!
         * \brief getBoxSize
         * \return bounding box size in a vec3i
         */
        virtual defaulttype::Vec3i getBoxSize() const = 0;

        /*!
         * \brief getBoxCoord
         * \param P : point in space
         * \return the box's coordinates (vec3i) containing point P
         */
        virtual defaulttype::Vec3i getBoxCoord(const defaulttype::Vector3 & P) const = 0;

        virtual void getElementSet(unsigned cx,unsigned cy, unsigned cz, std::set<unsigned> & selectElements) const = 0;

    };

    SOFA_ABSTRACT_CLASS(BaseGeometry,core::objectmodel::BaseObject);

    Data<defaulttype::Vector4> d_color;
    Data<double> d_drawScaleNormal;

    BaseGeometry()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , d_drawScaleNormal(initData(&d_drawScaleNormal, 1.0, "drawScaleNormal", "Color of the collision model")) {
        this->f_listening.setValue(true);
    }

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) = 0;

    inline const BaseGeometry * end() const {
        return this;
    }

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    void addBroadPhase(BroadPhase::SPtr d) {
        m_broadPhase.push_back(d);
        this->addSlave(d);
    }

    const std::vector<BroadPhase::SPtr> getBroadPhase() {
        return m_broadPhase;
    }

    virtual void prepareDetection() {}

    void handleEvent(sofa::core::objectmodel::Event *event) {
        if (! dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event)) return;

        for (unsigned i=0;i<m_broadPhase.size();i++) {
            m_broadPhase[i]->prepareDetection();
        }

        prepareDetection();
    }

protected:
    std::vector<BroadPhase::SPtr> m_broadPhase;
};

template<class DataTypes>
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

    SOFA_ABSTRACT_CLASS(SOFA_TEMPLATE(TBaseGeometry,DataTypes),BaseGeometry);

    core::objectmodel::SingleLink<TBaseGeometry<DataTypes>,State,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_state;
    Data<bool> drawCollision ;

    TBaseGeometry()
    : l_state(initLink("mstate", "link to state"))
    , drawCollision (initData(&drawCollision, false, "drawcollision", "draw collision")) {
        l_state.setPath("@.");
    }

    inline void drawNormals(const core::visual::VisualParams *vparams) {
        if (! vparams->displayFlags().getShowNormals() || d_drawScaleNormal.getValue() == 0.0) return;

        defaulttype::Vector4 color = d_color.getValue();
        color[3] = 1.0;

        for (auto it=this->begin();it!=this->end();it++) {
            BaseProximity::SPtr center = (*it)->center();
            vparams->drawTool()->drawArrow(
                center->getPosition(),
                center->getPosition() + center->getNormal() * d_drawScaleNormal.getValue(),
                d_drawScaleNormal.getValue() * 0.1,
                color
            );
        }
    }

    inline sofa::core::behavior::MechanicalState<DataTypes> * getState() const {
        return l_state.get();
    }

    template<class PROXIMITYDATA>
    inline void buildJacobianConstraint(const PROXIMITYDATA & data, core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & normals, double fact, unsigned constraintId) const {
        DataMatrixDeriv & c1_d = *cId[this->getState()].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (unsigned j=0;j<normals.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);
            data.addContributions(c_it, normals[j] * fact);
        }

        c1_d.endEdit();
    }

    inline void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, unsigned cid, const sofa::defaulttype::BaseVector* lambda) const {
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


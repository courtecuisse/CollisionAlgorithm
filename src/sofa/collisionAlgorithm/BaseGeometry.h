#pragma once

#include <sofa/simulation/CollisionBeginEvent.h>
#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/core/BehaviorModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/simulation/Visitor.h>
#include <sofa/gl/gl.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa {

namespace collisionAlgorithm {





/*!
 * \brief The BaseGeometry class is an abstract class defining a basic geometry
 * iterates through Proximity elements and draws them
 */
class BaseGeometry : public core::objectmodel::BaseObject
{
public:

    SOFA_ABSTRACT_CLASS(BaseGeometry,core::objectmodel::BaseObject);

    typedef sofa::Index Index;

    class BroadPhase : public core::objectmodel::BaseObject {
    public:

        SOFA_ABSTRACT_CLASS(BroadPhase,core::objectmodel::BaseObject);

        core::objectmodel::SingleLink<BroadPhase,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_geometry;

        /*!
         * \brief BroadPhase Constructor
         */
        BroadPhase()
        : l_geometry(initLink("geometry", "link to the geometry")) {}

        void init( ) override {
            if (l_geometry == NULL) return;
            l_geometry->setBroadPhase(this);
        }

        /*!
         * \brief prepareDetection virtual method to implement
         * detection pre-processing
         */
        virtual void prepareDetection() = 0;

        /*!
         * \brief getBoxSize
         * \return bounding box size in a vec3i
         */
        virtual type::Vec3i getBoxSize() const = 0;

        /*!
         * \brief getBoxCoord
         * \param P : point in space
         * \return the box's coordinates (vec3i) containing point P
         */
        virtual type::Vec3i getBoxCoord(const type::Vector3 & P) const = 0;

        virtual void getElementSet(type::Vec3i c, std::set<Index> & selectElements) const = 0;

    };

    Data<sofa::type::RGBAColor> d_color;
    Data<double> d_drawScaleNormal;

    BaseGeometry()
    : d_color(initData(&d_color, sofa::type::RGBAColor(1,0,1,1), "color", "Color of the collision model"))
    , d_drawScaleNormal(initData(&d_drawScaleNormal, 1.0, "drawScaleNormal", "Color of the collision model")) {
        this->f_listening.setValue(true);
    }

    virtual BaseElementIterator::UPtr begin(Index eid = 0) const = 0;

    inline const BaseGeometry * end() const {
        return this;
    }

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    void setBroadPhase(BroadPhase::SPtr d) {
        m_broadPhase = d;
        this->addSlave(d);
    }

    const BroadPhase::SPtr getBroadPhase() {
        return m_broadPhase;
    }

    virtual void prepareDetection() {}

    void handleEvent(sofa::core::objectmodel::Event *event) {
        if (! dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event)) return;

        prepareDetection();

        recomputeNormals();

        if (m_broadPhase != NULL) m_broadPhase->prepareDetection();
    }

    virtual void recomputeNormals() = 0;

protected:
    BroadPhase::SPtr m_broadPhase;
};


template<class PROXIMITYDATA>
class BaseNormalHandler {
public:
    virtual void updateNormals() = 0;

    virtual type::Vector3 computeNormal(const PROXIMITYDATA & data) const = 0;
};


template<class DataTypes,class TPROXIMITYDATA>
class TBaseGeometry : public BaseGeometry, public BaseNormalHandler<TPROXIMITYDATA> {
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
    typedef TPROXIMITYDATA PROXIMITYDATA;

    SOFA_ABSTRACT_CLASS(SOFA_TEMPLATE2(TBaseGeometry,DataTypes,TPROXIMITYDATA),BaseGeometry);

    core::objectmodel::SingleLink<TBaseGeometry<DataTypes,PROXIMITYDATA>,State,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_state;

    TBaseGeometry()
    : l_state(initLink("mstate", "link to state")) {
        l_state.setPath("@.");

        //By default the normal handle is the geometry
        setNormalHandler(this);
    }

    inline void drawNormals(const core::visual::VisualParams *vparams) {
        if (! vparams->displayFlags().getShowNormals() || d_drawScaleNormal.getValue() == 0.0) return;

        sofa::type::RGBAColor color = d_color.getValue();
        color[3] = 1.0;

        for (auto it=this->begin();it!=this->end();it++) {
            BaseProximity::SPtr center = (*it)->createProximity();
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
    inline void buildJacobianConstraint(const PROXIMITYDATA & data, core::MultiMatrixDerivId cId, const sofa::type::vector<type::Vector3> & normals, double fact, Index constraintId) const {
        DataMatrixDeriv & c1_d = *cId[this->getState()].write();
        MatrixDeriv & c1 = *c1_d.beginEdit();

        for (Index j=0;j<normals.size();j++) {
            MatrixDerivRowIterator c_it = c1.writeLine(constraintId+j);
            data.addContributions(c_it, normals[j] * fact);
        }

        c1_d.endEdit();
    }

    inline void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::defaulttype::BaseVector* lambda) const {
        auto res = sofa::helper::write(*resId[this->getState()].write());
        const typename DataTypes::MatrixDeriv& j = cParams->readJ(this->getState())->getValue();
        auto rowIt = j.readLine(cid_global+cid_local);
        const double f = lambda->element(cid_global+cid_local);
        for (auto colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt)
        {
            res[colIt.index()] += colIt.val() * f;
        }
    }


    /// Pre-construction check method called by ObjectFactory.
    /// Check that DataTypes matches the MechanicalState.
    template<class T>
    static bool canCreate(T*& obj, core::objectmodel::BaseContext* context, core::objectmodel::BaseObjectDescription* arg) {
        sofa::core::behavior::MechanicalState<DataTypes>* state = NULL;
        context->get(state);
        if (state == NULL) return false;
        return sofa::core::objectmodel::BaseObject::canCreate(obj, context, arg);
    }

    virtual std::string getTemplateName() const override {
        return templateName(this);
    }

    static std::string templateName(const TBaseGeometry<DataTypes,PROXIMITYDATA>* = NULL) {
        return DataTypes::Name();
    }

    virtual void updateNormals() override {}

    virtual void recomputeNormals() override {
        m_normalHandler->updateNormals();
    }

    virtual type::Vector3 getNormal(const PROXIMITYDATA & data) const {
        return m_normalHandler->computeNormal(data);
    }

    void setNormalHandler(BaseNormalHandler<TPROXIMITYDATA> * n) {
        if (n == NULL) return;

        m_normalHandler = n;
    }

    template<class PROXIMITYDATA>
    inline void buildConstraintMatrixJ0(const PROXIMITYDATA & data, int cId, sofa::defaulttype::BaseMatrix * J0, double fact) const {
        data.addColInJ0(cId, J0, fact);
    }

protected :
    BaseNormalHandler<TPROXIMITYDATA> * m_normalHandler; // this pointer cannot be NULL either it's the geometry or a handler

};

}

}


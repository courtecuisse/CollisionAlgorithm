#pragma once

#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/simulation/CollisionBeginEvent.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/core/BehaviorModel.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/simulation/Visitor.h>
#include <sofa/gl/gl.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/BaseElement.h>

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

    Data<sofa::type::RGBAColor> d_color;
    Data<double> d_drawScaleNormal;
    Data<bool> d_draw;

    BaseGeometry()
    : d_color(initData(&d_color, sofa::type::RGBAColor(1,0,1,1), "color", "Color of the collision model"))
    , d_drawScaleNormal(initData(&d_drawScaleNormal, 1.0, "drawScaleNormal", "Color of the collision model"))
    , d_draw(initData(&d_draw, true, "draw", "draw")) {
        this->f_listening.setValue(true);
    }

    virtual BaseElement::Iterator begin(Index eid = 0) const = 0;

    virtual const BaseOperations * getOperations() = 0;

    inline const BaseGeometry * end() const {
        return this;
    }

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    virtual void prepareDetection() {}

    void handleEvent(sofa::core::objectmodel::Event *event) {
        if (! dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event)) return;

        for (unsigned i=0;i<m_elements.size();i++) m_elements[i]->update();

        prepareDetection();
    }

    template<class ELEMENT, typename... Args>
    inline void createElement(Args...) {
//        typedef typename ELEMENT<PROXIMITY> ELEMENT_PRPOXIMITY;
        BaseElement::SPtr elmt(new ELEMENT(Args...));
        m_elements.push_back(elmt);
    }

private:
    std::vector<BaseElement::SPtr> m_elements;

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

    TBaseGeometry()
    : l_state(initLink("mstate", "link to state")) {
        l_state.setPath("@.");
    }

    inline void drawNormals(const core::visual::VisualParams *vparams) {
        if (! vparams->displayFlags().getShowNormals() || d_drawScaleNormal.getValue() == 0.0) return;

        sofa::type::RGBAColor color = d_color.getValue();
        color[3] = 1.0;

        auto createPorximityCenter = BaseOperations::createCenterProximity(getOperations());

        for (auto it=this->begin();it!=this->end();it++) {
            auto element = *it;
            BaseProximity::SPtr center = createPorximityCenter(element);
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

    static std::string templateName(const TBaseGeometry<DataTypes>* = NULL) {
        return DataTypes::Name();
    }

};

}

}


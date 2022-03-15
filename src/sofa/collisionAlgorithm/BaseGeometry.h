#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/ElementIterator.h>
#include <sofa/gl/gl.h>

namespace sofa ::collisionAlgorithm {

/*!
 * \brief The BaseGeometry class is an abstract class defining a basic geometry
 * iterates through Proximity elements and draws them
 */
class BaseGeometry : public CollisionComponent {
public:

    SOFA_ABSTRACT_CLASS(BaseGeometry,CollisionComponent);

    Data<sofa::type::RGBAColor> d_color;
    Data<double> d_drawScaleNormal;
    Data<bool> d_draw;

    BaseGeometry()
    : d_color(initData(&d_color, sofa::type::RGBAColor(1,0,1,1), "color", "Color of the collision model"))
    , d_drawScaleNormal(initData(&d_drawScaleNormal, 1.0, "drawScaleNormal", "Color of the collision model"))
    , d_draw(initData(&d_draw, true, "draw", "draw")) {
        this->f_listening.setValue(true);
    }

    virtual ElementIterator::SPtr begin() const = 0;

    inline const BaseGeometry * end() const { return this; }

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    void draw(const core::visual::VisualParams * vparams) override {
        type::RGBAColor color = d_color.getValue();

//        glDisable(GL_LIGHTING);
        if (color[3] == 0.0) return;

        glColor4f(color[0],color[1],color[2],color[3]);
        for (auto it = begin();it != end(); it++) {
            it->element()->draw(vparams);
        }

        if (! vparams->displayFlags().getShowNormals()) return;
        double scale = d_drawScaleNormal.getValue();
        if (scale == 0.0) return;
        for (auto it = begin();it != end(); it++) {
            std::vector<BaseProximity::SPtr> res;
            it->element()->getControlProximities(res);
            for (unsigned i=0;i<res.size();i++) {
                BaseProximity::SPtr center = res[i];

                vparams->drawTool()->drawArrow(
                    center->getPosition(),
                    center->getPosition() + center->getNormal() * scale,
                    scale * 0.1,
                    color);
            }
        }
    }

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


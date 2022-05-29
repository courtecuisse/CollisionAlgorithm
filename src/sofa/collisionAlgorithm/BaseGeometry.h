#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/proximity/MechanicalProximity.h>
#include <sofa/collisionAlgorithm/ElementIterator.h>
#include <sofa/gl/gl.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>

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

    void init() {
        m_pointElements.clear();
        m_edgeElements.clear();
        m_triangleElements.clear();
        m_tetrahedronElements.clear();

        buildPointElements();
        buildEdgeElements();
        buildTriangleElements();
        buildTetrahedronElements();
        prepareDetection();
    }

    virtual void buildPointElements() {}

    virtual void buildEdgeElements() {}

    virtual void buildTriangleElements() {}

    virtual void buildTetrahedronElements() {}

    virtual void prepareDetection() {
        for (unsigned i=0;i<m_pointElements.size();i++) m_pointElements[i]->update();
        for (unsigned i=0;i<m_edgeElements.size();i++) m_edgeElements[i]->update();
        for (unsigned i=0;i<m_triangleElements.size();i++) m_triangleElements[i]->update();
        for (unsigned i=0;i<m_tetrahedronElements.size();i++) m_tetrahedronElements[i]->update();
    }

    virtual ElementIterator::SPtr begin(unsigned id = 0) const = 0;

    inline const BaseGeometry * end() const { return this; }

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

    void draw(const core::visual::VisualParams * vparams) override {
        if (! vparams->displayFlags().getShowCollisionModels()) return;
        if (! d_draw.getValue()) return;
        type::RGBAColor color = d_color.getValue();

        glDisable(GL_LIGHTING);
        if (color[3] == 0.0) return;

        glColor4f(color[0],color[1],color[2],color[3]);
        for (auto it = begin();it != end(); it++) {
            it->element()->draw(vparams);
        }        
    }

    inline ElementIterator::SPtr pointBegin(unsigned id = 0) const {
        return ElementIterator::SPtr(new TDefaultElementIteratorPtr(m_pointElements,id));
    }

    inline ElementIterator::SPtr edgeBegin(unsigned id = 0) const {
        return ElementIterator::SPtr(new TDefaultElementIteratorPtr(m_edgeElements,id));
    }

    inline ElementIterator::SPtr triangleBegin(unsigned id = 0) const {
        return ElementIterator::SPtr(new TDefaultElementIteratorPtr(m_triangleElements,id));
    }

    inline ElementIterator::SPtr tetrahedronBegin(unsigned id = 0) const {
        return ElementIterator::SPtr(new TDefaultElementIteratorPtr(m_tetrahedronElements,id));
    }

protected:

    virtual ElementContainer<PointElement::SPtr> & pointElements() { return m_pointElements; }

    virtual ElementContainer<EdgeElement::SPtr> & edgeElements() { return m_edgeElements; }

    virtual ElementContainer<TriangleElement::SPtr> & triangleElements() { return m_triangleElements; }

    virtual ElementContainer<TetrahedronElement::SPtr> & tetrahedronElements() { return m_tetrahedronElements; }

private:

    ElementContainer<PointElement::SPtr> m_pointElements;
    ElementContainer<EdgeElement::SPtr> m_edgeElements;
    ElementContainer<TriangleElement::SPtr> m_triangleElements;
    ElementContainer<TetrahedronElement::SPtr> m_tetrahedronElements;
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

    SOFA_CLASS(SOFA_TEMPLATE(TBaseGeometry,DataTypes),BaseGeometry);

    core::objectmodel::SingleLink<TBaseGeometry<DataTypes>,State,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_state;

    TBaseGeometry()
    : l_state(initLink("mstate", "link to state")) {
        l_state.setPath("@.");
//        for (unsigned j=0; j<this->getState()->getSize(); j++) {
//            m_topoProx.push_back(TBaseProximity<DataTypes>::template create<TopologyProximity<DataTypes>>(l_state, j));
// //            m_topoProx.push_back(BaseProximity::create<TopologyProximity<DataTypes>>(l_state, j));
//        }
    }



//    virtual void buildTopologyProximity() {
//        this->m_topoProx.clear();
//        for (unsigned j=0; j<this->getState()->getSize(); j++) {
////            m_topoProx.push_back(TBaseProximity<DataTypes>::template create<TopologyProximity<DataTypes>>(l_state, j));
//             m_topoProx.push_back(BaseProximity::create<TopologyProximity<DataTypes>>(l_state, j));
//        }
//    }




    inline sofa::core::behavior::MechanicalState<DataTypes> * getState() const {
        return l_state.get();
    }

//    inline std::vector<typename TopologyProximity<DataTypes>::SPtr> & getTopoProx(){
//        return m_topoProx;
//    }

//    inline typename TopologyProximity<DataTypes>::SPtr getTopoProxIdx(unsigned i){
//        return m_topoProx[i];
//    }

//	inline std::vector<BaseElement::SPtr > getBaseElements() = 0;

    inline void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId resId, Index cid_global, Index cid_local, const sofa::linearalgebra::BaseVector* lambda) const {
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


#pragma once

#include <collisionAlgorithm.h>
#include <memory>
#include <map>
#include <vector>
#include <BaseElement.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/simulation/Node.h>

namespace sofa {

template<class T>
class DataLink : public sofa::core::objectmodel::BaseLink, public sofa::simulation::MutationListener {
public:
    typedef sofa::simulation::Node Node;
    typedef void (core::objectmodel::BaseObject::*BaseObjectCallBack)();

    Data<std::string> m_path;

    DataLink(const char * name, core::objectmodel::BaseObject * obj)
    : BaseLink(FLAG_DATALINK)
    , m_path(obj->initData(&m_path, name, "path To the link"))
    , m_object(obj)
    , m_link(NULL)
    , m_callBack(NULL) {
        m_path.getOwner()->addLink(this);
        updateLinks();
    }

    template<class F>
    void setCallback(const F c) {
        m_callBack = reinterpret_cast<BaseObjectCallBack>(c);
    }

    inline T* operator->() const {
        return m_link;
    }

    inline T* get() const {
        return m_link;
    }

    friend inline bool operator==(const DataLink & lhs, const T * rhs) {
         return lhs.m_link == rhs;
     }

    friend inline bool operator!=(const DataLink & lhs, const T * rhs) {
         return lhs.m_link != rhs;
     }

private:

    sofa::core::objectmodel::Base* getOwnerBase() const { return NULL; }

    sofa::core::objectmodel::BaseData* getData() const { return m_path.getData(); }

    sofa::core::objectmodel::BaseData* getOwnerData() const { return m_path.getData(); }

    size_t getSize() const { return 0; }

    sofa::core::objectmodel::Base* getLinkedBase(unsigned int ) const { return NULL; }

    sofa::core::objectmodel::BaseData* getLinkedData(unsigned int ) const { return NULL; }

    std::string getLinkedPath(unsigned int ) const { return m_path.getValue(); }

    const sofa::core::objectmodel::BaseClass* getDestClass() const { return NULL; }

    const sofa::core::objectmodel::BaseClass* getOwnerClass() const { return NULL; }

    void copyAspect(int , int ) { }

    bool read( const std::string& ) { return true; }

    bool updateLinks() {
        m_link = NULL;
        if  (Node * n = dynamic_cast<sofa::simulation::Node*>(m_object->getContext())) {
            n->removeListener(this);
            n->addListener(this);
        }

        return true;
    }

    void getLink() {
        std::cout << "GETLINK" << std::endl;
        m_link = m_object->getContext()->get<T>(m_path.getValue());

        std::cout << "LINK:" << m_link << std::endl;
        if (m_link != NULL) m_path.setValue(m_link->getName());

        if (m_callBack != NULL) (m_object->*m_callBack)();
    }

    void addChild(Node* , Node* ) { getLink(); }

    void removeChild(Node* , Node* ) { getLink(); }

    void moveChild(Node* , Node* , Node* ) { getLink(); }

    void addObject(Node* , core::objectmodel::BaseObject* ) { getLink(); }

    void removeObject(Node* , core::objectmodel::BaseObject* ) { getLink(); }

    void moveObject(Node* , Node* , core::objectmodel::BaseObject* ) { getLink(); }

    void addSlave(core::objectmodel::BaseObject* , core::objectmodel::BaseObject* ) { getLink(); }

    void removeSlave(core::objectmodel::BaseObject* , core::objectmodel::BaseObject* ) { getLink(); }

    void moveSlave(core::objectmodel::BaseObject* , core::objectmodel::BaseObject* , core::objectmodel::BaseObject* ) { getLink(); }

    void sleepChanged(Node* ) { getLink(); }

protected:
    core::objectmodel::BaseObject * m_object;
    T* m_link;
    BaseObjectCallBack m_callBack;
};

namespace collisionAlgorithm {

class BaseGeometry : public core::objectmodel::BaseObject {
public:

    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;

    DataLink<sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> > d_state;

    BaseGeometry()
    : d_state("mstate", this) {
        m_dirty = true;

        d_state.setCallback(&BaseGeometry::newState);
    }

    void newState() {
        m_elements.clear();
    }

    virtual void handleEvent(core::objectmodel::Event * e) {
        if (dynamic_cast<simulation::AnimateBeginEvent *>(e)) m_dirty=true;
    }

    unsigned getNbElements() {
        if (m_dirty) {
            prepareDetection();
            m_dirty = false;
        }

        return (unsigned) m_elements.size();
    }

    ConstraintElementPtr getElement(unsigned i) {
        if (m_dirty) {
            prepareDetection();
            m_dirty = false;
        }

        return m_elements[i];
    }

//    ConstraintProximityPtr project(const Vector3 & P) {
//        double min_dist = std::numeric_limits<double>::max();
//        ConstraintProximityPtr min_prox = NULL;

//        for (unsigned i=0;i<m_elements.size();i++) {
//            ConstraintProximityPtr pdest = m_elements[i]->project(P);
//            double dist = (P - pdest->getPosition()).norm();
//            if (dist<min_dist) {
//                min_dist = dist;
//                min_prox = pdest;
//            }
//        }

//        return min_prox;
//    }

    virtual sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes> * getState() {
        return d_state.get();
    }

    void draw(const core::visual::VisualParams *vparams) {
        if (! vparams->displayFlags().getShowCollisionModels()) return;

        glDisable(GL_LIGHTING);

        for(unsigned i=0;i<m_elements.size();i++) {
            m_elements[i]->draw(vparams);
        }
    }

protected:
    std::vector<ConstraintElementPtr> m_elements;
    bool m_dirty;

    virtual void prepareDetection() {}
};

}

}

#pragma once

#include <sofa/helper/system/config.h>
#include <sofa/simulation/Node.h>
#include <GL/gl.h>

namespace sofa {

namespace collisionAlgorithm {

#ifndef MSOFA
template<class T>
class DataLink : public sofa::core::objectmodel::BaseLink, public sofa::simulation::MutationListener {
public:
    typedef sofa::simulation::Node Node;

    Data<std::string> m_path;

    DataLink(const char * name, core::objectmodel::BaseObject * obj)
    : BaseLink(FLAG_DATALINK)
    , m_path(obj->initData(&m_path, name, "path To the link"))
    , m_object(obj)
    , m_link(NULL) {
        m_path.addLink(this);
        updateLinks();
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
        m_link = m_object->getContext()->get<T>(m_path.getValue());

        if (m_link != NULL) m_path.setValue(m_link->getName());


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
};
#endif

}

}


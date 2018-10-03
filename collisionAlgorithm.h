/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef INITMISCPLUGIN_H
#define INITMISCPLUGIN_H

#include <sofa/helper/system/config.h>
#include <sofa/simulation/Node.h>


#ifdef SOFA_BUILD_PLUGINEXAMPLE
#define SOFA_COLLISIONALGORITHMPLUGIN_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#define SOFA_COLLISIONALGORITHMPLUGIN_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

namespace sofa {

namespace collisionAlgorithm {

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
    , m_link(NULL) {
        m_path.getOwner()->addLink(this);
        updateLinks();
    }

    template<class F>
    void addCallback(const F c) {
        m_callBack.push_back(reinterpret_cast<BaseObjectCallBack>(c));
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

        for (unsigned i=0;i<m_callBack.size();i++) {
            (m_object->*m_callBack[i])();
        }
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
    std::vector<BaseObjectCallBack> m_callBack;
};

}

}

#endif

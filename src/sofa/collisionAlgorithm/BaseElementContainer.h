#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BroadPhase;

class BaseElementContainer {
public:

    BaseElementContainer()
    : m_broadPhase(NULL) {}

    typedef sofa::core::objectmodel::TClass<BaseElementContainer,sofa::core::objectmodel::BaseData> MyClass;

    static const MyClass* GetClass() { return MyClass::get(); }
    virtual const sofa::core::objectmodel::BaseClass* getClass() const
    { return GetClass(); }

    template<class T>
    static void dynamicCast(T*& ptr, sofa::core::objectmodel::Base* b) {
        ptr = dynamic_cast<T*>(b);
    }

    template<class T>
    static std::string className(const T* ptr= NULL) {
        return sofa::core::objectmodel::BaseClass::defaultClassName(ptr);
    }

    template<class T>
    static std::string namespaceName(const T* ptr= NULL) {
        return sofa::core::objectmodel::BaseClass::defaultNamespaceName(ptr);
    }

    template<class T>
    static std::string templateName(const T* ptr= NULL) {
        return sofa::core::objectmodel::BaseClass::BaseClass::defaultTemplateName(ptr);
    }

    template< class T>
    static std::string shortName( const T* ptr = NULL, sofa::core::objectmodel::BaseObjectDescription* = NULL ) {
        std::string shortname = T::className(ptr);
        if( !shortname.empty() )
        {
            *shortname.begin() = ::tolower(*shortname.begin());
        }
        return shortname;
    }

    void setBroadPhase(BroadPhase * d) {
        m_broadPhase = d;
    }

    void unsetBroadPhase(BroadPhase * d) {
        if (m_broadPhase == d) m_broadPhase = NULL;
    }

    const BroadPhase * getBroadPhase() {
        return m_broadPhase;
    }

    virtual sofa::core::objectmodel::BaseData* getData() const  = 0;

    virtual sofa::core::objectmodel::Base* getOwner() const = 0;

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) = 0;

    virtual unsigned end() const = 0;

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

protected:
    BroadPhase * m_broadPhase;

};

}

namespace core {

namespace objectmodel {

template<>
class LinkTraitsPtrCasts<collisionAlgorithm::BaseElementContainer>
{
public:
    static sofa::core::objectmodel::Base* getBase(collisionAlgorithm::BaseElementContainer* n) {
        if (!n) return NULL;
        return n->getOwner();
    }

    static sofa::core::objectmodel::BaseData* getData(collisionAlgorithm::BaseElementContainer* n) {
        if (!n) return NULL;
        return n->getData();
    }
};

}

}

}


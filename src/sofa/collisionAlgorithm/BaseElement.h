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
    : m_broadPhase(NULL)
    , m_update_time(-1.0)  {}

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
        updateInternalData();
        return m_broadPhase;
    }

    virtual sofa::core::objectmodel::BaseData* getData() const  = 0;

    virtual sofa::core::objectmodel::Base* getOwner() const = 0;

    virtual BaseProximity::SPtr project(const BaseElementIterator * it, const defaulttype::Vector3 & P) const = 0;

    virtual BaseProximity::SPtr center(const BaseElementIterator * it) const = 0;

    virtual defaulttype::BoundingBox getBBox(const BaseElementIterator * it) const = 0;

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) = 0;

    virtual unsigned end() const = 0;

    virtual void init() {}

    virtual void prepareDetection() {}

    virtual double getTime() const = 0;

    virtual sofa::core::behavior::BaseMechanicalState * getState() const = 0;

protected:
    BroadPhase * m_broadPhase;
    double m_update_time;

    inline void updateInternalData() {
        double time = getTime();

        if (m_update_time < 0) init();

        if (m_update_time < time) {
            m_update_time = time;
            prepareDetection();
//            if (m_broadPhase != NULL) m_broadPhase->prepareDetection();
        }
    }
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


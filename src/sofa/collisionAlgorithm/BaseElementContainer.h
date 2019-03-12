#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/BehaviorModel.h>
#include <sofa/core/collision/Pipeline.h>
#include <sofa/core/topology/BaseMeshTopology.h>
#include <memory>
#include <map>
#include <vector>
#include <qopengl.h>

namespace sofa
{

namespace collisionAlgorithm
{

class BroadPhase;
class BaseGeometry;

class BaseDataElmtContainer {
public:

    class  ElementOwner {
    public:
        void init() {
            std::cout << "INIT" << std::endl;
            for (auto it=m_containers.cbegin();it!=m_containers.cend();it++) {
                (*it)->init();
                (*it)->prepareDetection();
            }
        }

        void prepareDetection() {
            std::cout << "UPDATE" << std::endl;
            for (auto it=m_containers.cbegin();it!=m_containers.cend();it++) {
                (*it)->prepareDetection();
            }
        }

        void registerContainer(BaseDataElmtContainer*c) {
            std::cout << "REGISTER" << std::endl;
            m_containers.insert(c);
        }

    protected:
        std::set<BaseDataElmtContainer*> m_containers;
    };

    typedef sofa::core::objectmodel::TClass<BaseDataElmtContainer,sofa::core::objectmodel::BaseData> MyClass;
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

    virtual sofa::core::objectmodel::Base* getOwner() const = 0;

    virtual sofa::core::objectmodel::BaseData* getData() const = 0;

    void setBroadPhase(BroadPhase * d) {
        m_broadPhase = d;
    }

    void unsetBroadPhase(BroadPhase * d) {
        if (m_broadPhase == d) m_broadPhase = NULL;
    }

    BroadPhase * getBroadPhase() const {
        return m_broadPhase;
    }

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const  = 0;

    virtual const BaseGeometry * end() const = 0;

    virtual void init() {}

    virtual void prepareDetection() {}

protected:
    BroadPhase * m_broadPhase;
};

template<class ELMT>
class DataElemntContainer : public core::objectmodel::Data<helper::vector<ELMT> >, public BaseDataElmtContainer {
public:

    explicit DataElemntContainer(const typename core::objectmodel::Data<helper::vector<ELMT> >::InitData& init)
    : Data<helper::vector<ELMT>>(init) {
        if (ElementOwner* elmt = dynamic_cast<ElementOwner*>(init.owner)) elmt->registerContainer(this);
        m_broadPhase = NULL;
    }

    virtual sofa::core::objectmodel::Base* getOwner() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::getOwner();
    }

    virtual sofa::core::objectmodel::BaseData* getData() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::getData();
    }

};

}

namespace core {

namespace objectmodel {

template<>
class LinkTraitsPtrCasts<collisionAlgorithm::BaseDataElmtContainer>
{
public:
    static sofa::core::objectmodel::Base* getBase(collisionAlgorithm::BaseDataElmtContainer* n) {
        if (!n) return NULL;
        return n->getOwner();
    }

    static sofa::core::objectmodel::BaseData* getData(collisionAlgorithm::BaseDataElmtContainer* n) {
        if (!n) return NULL;
        return n->getData();
    }
};

}

}

}


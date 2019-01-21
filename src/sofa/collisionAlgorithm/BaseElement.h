#pragma once

#include <sofa/collisionAlgorithm/BaseElementIterator.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

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

class BaseDataElmt {
public:

    typedef sofa::core::objectmodel::TClass<BaseDataElmt,sofa::core::objectmodel::BaseData> MyClass;
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

    virtual unsigned size() const = 0;

    virtual sofa::core::objectmodel::Base* getOwner() const = 0;

    virtual sofa::core::objectmodel::BaseData* getData() const = 0;

    void setBroadPhase(BroadPhase * d) {
        m_broadPhase = d;
    }

    void unsetBroadPhase(BroadPhase * d) {
        if (m_broadPhase == d) m_broadPhase = NULL;
    }

    const BroadPhase * getBroadPhase() const {
        return m_broadPhase;
    }

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const  = 0;

    virtual const BaseGeometry * end() const = 0;

protected:
    BroadPhase * m_broadPhase;
};

template<class ELMT>
class DataElemnt : public core::objectmodel::Data<helper::vector<ELMT> >, public BaseDataElmt {
public:

    explicit DataElemnt(const typename core::objectmodel::Data<helper::vector<ELMT> >::InitData& init)
    : Data<helper::vector<ELMT>>(init) {
        m_broadPhase = NULL;
    }

    virtual bool read(const std::string& value) {
        return core::objectmodel::Data<helper::vector<ELMT> >::read(value);
    }

    virtual void copyAspect(int destAspect, int srcAspect) {
        core::objectmodel::Data<helper::vector<ELMT> >::copyAspect(destAspect,srcAspect);
    }

    virtual void printValue(std::ostream& os) const {
        core::objectmodel::Data<helper::vector<ELMT> >::printValue(os);
    }

    virtual std::string getValueString() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::getValueString();
    }

    virtual std::string getValueTypeString() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::getValueTypeString();
    }

    virtual const sofa::defaulttype::AbstractTypeInfo* getValueTypeInfo() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::getValueTypeInfo();
    }

    virtual const void* getValueVoidPtr() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::getValueVoidPtr();
    }

    virtual void* beginEditVoidPtr() {
        return core::objectmodel::Data<helper::vector<ELMT> >::beginEditVoidPtr();
    }

    virtual void endEditVoidPtr() {
        core::objectmodel::Data<helper::vector<ELMT> >::endEditVoidPtr();
    }

    virtual void releaseAspect(int aspect) {
        core::objectmodel::Data<helper::vector<ELMT> >::releaseAspect(aspect);
    }

    virtual bool isCounterValid() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::isCounterValid();
    }

    virtual sofa::core::objectmodel::Base* getOwner() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::getOwner();
    }

    virtual sofa::core::objectmodel::BaseData* getData() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::getData();
    }

    unsigned size() const {
        return this->getValue().size();
    }

    virtual void init() {
        if (this->m_broadPhase) this->m_broadPhase->init();
    }

    virtual void prepareDetection() {
        if (this->m_broadPhase) this->m_broadPhase->prepareDetection();
    }

};

}

namespace core {

namespace objectmodel {

template<>
class LinkTraitsPtrCasts<collisionAlgorithm::BaseDataElmt>
{
public:
    static sofa::core::objectmodel::Base* getBase(collisionAlgorithm::BaseDataElmt* n) {
        if (!n) return NULL;
        return n->getOwner();
    }

    static sofa::core::objectmodel::BaseData* getData(collisionAlgorithm::BaseDataElmt* n) {
        if (!n) return NULL;
        return n->getData();
    }
};

}

}

}


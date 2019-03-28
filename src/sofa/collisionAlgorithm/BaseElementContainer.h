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

/*!
 * \brief The BaseDataElmtContainer class is an abstract data containing class
 */
class BaseDataElmtContainer {
public:

    /*!
     * \brief The ElementOwner class is a visitor that processes
     * an std::set of BaseDataElmtContainer for collision detection
     */
    class  ElementOwner {
    public:
        /*!
         * \brief init iterates through the containers, initializing them
         * and preparing for detection
         */
        void init() {
            std::cout << "INIT" << std::endl;
            for (auto it=m_containers.cbegin();it!=m_containers.cend();it++) {
                (*it)->init();
                (*it)->prepareDetection();
            }
        }

        /*!
         * \brief prepareDetection updates detection preparation
         */
        void prepareDetection() {
            std::cout << "UPDATE" << std::endl;
            for (auto it=m_containers.cbegin();it!=m_containers.cend();it++) {
                (*it)->prepareDetection();
            }
        }

        /*!
         * \brief registerContainer adds a new container to process
         * \param c : Container to add to std::set
         */
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

    /*!
     * \brief setBroadPhase sets broad phase to parameter
     * \param d : broad phase to use
     */
    void setBroadPhase(BroadPhase * d) {
        m_broadPhase = d;
    }

    /*!
     * \brief unsetBroadPhase sets broadphase member
     * to NULL if it is equal to the passed parameter
     * \param d : broad phase to compare to
     */
    void unsetBroadPhase(BroadPhase * d) {
        if (m_broadPhase == d) m_broadPhase = NULL;
    }

    /*!
     * \brief getBroadPhase accessor
     * \return const broadphase member
     */
    BroadPhase * getBroadPhase() const {
        return m_broadPhase;
    }

    /*!
     * \brief begin should define the beginning of an iterator
     * \param eid
     * \return an iterator
     */
    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const  = 0;

    /*!
     * \brief end marks the end of an iterator
     * \return
     */
    virtual const BaseGeometry * end() const = 0;

    /*!
     * \brief init iniatializes the object before preparing it for detection
     */
    virtual void init() {}

    /*!
     * \brief prepareDetection prepares for detection
     */
    virtual void prepareDetection() {}

protected:
    BroadPhase * m_broadPhase;
};

/*!
 * \class DataElementContainer
 * \brief Inherits Data<> and BaseDataElmtContainer.
 * Is still abstract, doesn't implement all functionalities
 */
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


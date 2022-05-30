#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa::collisionAlgorithm::Operations {

template<typename CHILD, class TFUNC>
class GenericOperation {
    friend CHILD;

public:

    typedef TFUNC FUNC;

    template<class T>
    static inline FUNC get(T * obj) { return get(obj->getOperationsHash()); }

    static FUNC get(size_t id/*const ElementIterator::SPtr itelmt*/) {
        //size_t id = itelmt->getOperationsHash();
        auto it = getSingleton()->m_map.find(id);
        if (it == getSingleton()->m_map.end()) return getSingleton()->getDefault();
        return it->second;
    }

    template<class ELMT>
    static int register_func(FUNC f) {
        size_t id = typeid(ELMT).hash_code();
        auto it = getSingleton()->m_map.find(id);
        if (it != getSingleton()->m_map.end()) std::cerr << "createCenterPointProximity with operation " << id << " already in the factory" << std::endl;
        getSingleton()->m_map[id] = f;
        return 0;
    }

    virtual FUNC getDefault() const = 0;

private:
    std::map<size_t,FUNC> m_map;

    static GenericOperation * getSingleton() {
        static GenericOperation * m_singleton = NULL;
        if (m_singleton == NULL) m_singleton = new CHILD();
        return m_singleton;
    }

    //No construction of the is is allowed
    GenericOperation() {}
};



template<typename CHILD, class TFUNC>
class GenericOperation2 {
    friend CHILD;

public:

    typedef TFUNC FUNC;

    static FUNC get(size_t id1, size_t id2 /*const ElementIterator::SPtr itelmt1,const ElementIterator::SPtr itelmt2*/) {
//        size_t id = (itelmt1->getOperationsHash()&(0xFFFFFFFF00000000)) | (itelmt2->getOperationsHash()&0x00000000FFFFFFFF);
//        size_t id = itelmt1->getOperationsHash() * itelmt1->getOperationsHash() + itelmt2->getOperationsHash();
        size_t id = id1 * id1 + id2;
        auto it = getSingleton()->m_map.find(id);
        if (it == getSingleton()->m_map.end()) return getSingleton()->getDefault();
        return it->second;
    }

    template<class ELMT1,class ELMT2>
    static int register_func(FUNC f) {
//        size_t id = (typeid(ELMT1).hash_code()&(0xFFFFFFFF00000000)) | (typeid(ELMT2).hash_code()&0x00000000FFFFFFFF);
        size_t id = typeid(ELMT1).hash_code() * typeid(ELMT1).hash_code() + typeid(ELMT2).hash_code();
        auto it = getSingleton()->m_map.find(id);
        if (it != getSingleton()->m_map.end()) std::cerr << "Intersect with operation " << id << " already in the factory" << std::endl;
        getSingleton()->m_map[id] = f;
        return 0;
    }

    virtual FUNC getDefault() const = 0;

private:
    std::map<size_t,FUNC> m_map;

    static GenericOperation2 * getSingleton() {
        static GenericOperation2 * m_singleton = NULL;
        if (m_singleton == NULL) m_singleton = new CHILD();
        return m_singleton;
    }

    //No construction of the is is allowed
    GenericOperation2() {}
};

}

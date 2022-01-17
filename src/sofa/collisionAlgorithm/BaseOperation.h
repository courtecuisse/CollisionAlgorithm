#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa::collisionAlgorithm::Operations {

template<typename CHILD, class TFUNC>
class GenericOperation {
    friend CHILD;

public:

    typedef TFUNC FUNC;

    static FUNC func(const BaseGeometry * geo) {
        size_t id = geo->getOperationsHash();
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

}

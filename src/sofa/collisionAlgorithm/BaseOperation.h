#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa::collisionAlgorithm::Operations {

template<class TFUNC>
class GenericOperation {
public:

    typedef TFUNC FUNC;

    static FUNC func(const BaseGeometry * geo) {
        size_t id = geo->getOperationsHash();
        auto it = m_singleton.m_map.find(id);
        if (it == m_singleton.m_map.end()) return m_singleton.getDefault();
        return it->second;
    }

    template<class ELMT>
    static int register_func(FUNC f) {
        size_t id = typeid(ELMT).hash_code();
        auto it = m_singleton.m_map.find(id);
        if (it != m_singleton.m_map.end()) std::cerr << "createCenterPointProximity with operation " << id << " already in the factory" << std::endl;
        m_singleton.m_map[id] = f;
        return 0;
    }

    //This function must be overloaded in all the operations !!!
    //The code below makes no sens and will probably result in a sefgault unless the function is overloaded
    //It cannot be made virtual pure since the singleton is intantiated in this class
    virtual FUNC getDefault() const {
        std::cerr << "YOU MUST DEFINE THE DEFUALT OPERATION" << std::endl;
        return FUNC();
    }

private:
    std::map<size_t,FUNC> m_map;
    static GenericOperation m_singleton;

    //No construction of the is is allowed
    GenericOperation() {}
};

}

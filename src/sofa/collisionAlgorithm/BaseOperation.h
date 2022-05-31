#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/helper/NameDecoder.h>

namespace sofa::collisionAlgorithm::Operations {

template<typename CHILD, class DEFAULT_RETURN, class... PARAMS>
class GenericOperation {
public:
    friend CHILD;

    typedef DEFAULT_RETURN (*FUNC)(PARAMS...);

    template<class RESTYPE = DEFAULT_RETURN>
    static RESTYPE (*get(const std::type_info & id)) (PARAMS...) {
        typedef RESTYPE (*function_type)(PARAMS...);

        //size_t id = itelmt->getTypeInfo();
        auto it_type = getSingleton()->m_map.find(id.hash_code());
        if (it_type == getSingleton()->m_map.end()) return &GenericOperation::t_defaultFunc<RESTYPE>;

        auto it_res = it_type->second.find(typeid(RESTYPE).hash_code());
        if (it_res == it_type->second.end()) return &GenericOperation::t_defaultFunc<RESTYPE>;


        return *((function_type*) &it_res->second);
    }

    template<class ELMT,class RES_TYPE = DEFAULT_RETURN>
    static int register_func(RES_TYPE (*f)(PARAMS...)) {
        size_t hash_type = typeid(ELMT).hash_code();
        size_t hash_res = typeid(RES_TYPE).hash_code();

        std::map<size_t,void*> & map_type = getSingleton()->m_map[hash_type];

        auto it_res = map_type.find(hash_res);
        if (it_res != map_type.cend()) {
            std::cerr << "createCenterPointProximity with operation " << sofa::helper::NameDecoder::decodeClassName(typeid(ELMT)) << " with return " << sofa::helper::NameDecoder::decodeClassName(typeid(RES_TYPE)) << " already in the factory" << std::endl;
            return 1;
        }

        map_type[hash_res] = (void*) f;

        return 0;
    }

    virtual DEFAULT_RETURN defaultFunc(PARAMS...) const = 0;

private:
    std::map<size_t,std::map<size_t,void*>> m_map;

    static GenericOperation * getSingleton() {
        static GenericOperation * m_singleton = NULL;
        if (m_singleton == NULL) m_singleton = new CHILD();
        return m_singleton;
    }

    template<class RESTYPE>
    static RESTYPE t_defaultFunc(PARAMS... params) {
        return getSingleton()->defaultFunc(params...);
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
//        size_t id = (itelmt1->getTypeInfo()&(0xFFFFFFFF00000000)) | (itelmt2->getTypeInfo()&0x00000000FFFFFFFF);
//        size_t id = itelmt1->getTypeInfo() * itelmt1->getTypeInfo() + itelmt2->getTypeInfo();
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

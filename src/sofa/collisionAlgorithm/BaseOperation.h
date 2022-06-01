#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/helper/NameDecoder.h>

namespace sofa::collisionAlgorithm::Operations {

template<typename CHILD, class DEFAULT_RETURN, class... PARAMS>
class GenericOperation {
public:
    friend CHILD;

    typedef DEFAULT_RETURN (*FUNC)(PARAMS...);
//    typedef std::pair<const std::type_info &,const std::type_info &> KEY_MAP; // <RETURN,TYPE>
    typedef std::pair<size_t,size_t> KEY_MAP; // <RETURN,TYPE>

    template<class RESTYPE = DEFAULT_RETURN>
    static RESTYPE (*get(const std::type_info & id)) (PARAMS...) {
        typedef RESTYPE (*function_type)(PARAMS...);

        KEY_MAP key(typeid(RESTYPE).hash_code(),id.hash_code());

        //size_t id = itelmt->getTypeInfo();
        auto it_type = getSingleton()->m_map.find(key);
        if (it_type == getSingleton()->m_map.cend()) return &GenericOperation::t_defaultFunc<RESTYPE>;

        return *((function_type*) &it_type->second);
    }

    template<class ELMT,class RES_TYPE = DEFAULT_RETURN>
    static int register_func(RES_TYPE (*f)(PARAMS...)) {
        KEY_MAP key(typeid(RES_TYPE).hash_code(),typeid(ELMT).hash_code());

        auto it = getSingleton()->m_map.find(key);
        if (it != getSingleton()->m_map.cend()) {
            std::cerr << sofa::helper::NameDecoder::decodeClassName(typeid(CHILD)) << " with operation " << sofa::helper::NameDecoder::decodeClassName(typeid(ELMT)) << " with return " << sofa::helper::NameDecoder::decodeClassName(typeid(RES_TYPE)) << " already in the factory" << std::endl;
            return 1;
        }

        getSingleton()->m_map[key] = (void*) f;

        return 0;
    }

    virtual DEFAULT_RETURN defaultFunc(PARAMS...) const = 0;

private:
    std::map<KEY_MAP,void*> m_map;

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



template<typename CHILD, class DEFAULT_RETURN, class... PARAMS>
class GenericOperation2 {
public:
    friend CHILD;

    typedef DEFAULT_RETURN (*FUNC)(PARAMS...);
    typedef std::tuple<size_t,size_t,size_t> KEY_MAP; // <RETURN,TYPE1,TYPE2>

    template<class RESTYPE = DEFAULT_RETURN>
    static RESTYPE (*get(const std::type_info & id1, const std::type_info & id2)) (PARAMS...) {
        typedef RESTYPE (*function_type)(PARAMS...);

        KEY_MAP key(typeid(RESTYPE).hash_code(),id1.hash_code(),id2.hash_code());

        //size_t id = itelmt->getTypeInfo();
        auto it = getSingleton()->m_map.find(key);
        if (it == getSingleton()->m_map.cend()) return &GenericOperation2::t_defaultFunc<RESTYPE>;

        return *((function_type*) &it->second);
    }

    template<class ELMT1,class ELMT2,class RES_TYPE = DEFAULT_RETURN>
    static int register_func(FUNC f) {
        KEY_MAP key(typeid(ELMT1).hash_code(),typeid(ELMT2).hash_code(),typeid(RES_TYPE).hash_code());

        auto it = getSingleton()->m_map.find(key);
        if (it != getSingleton()->m_map.cend()) {
            std::cerr << sofa::helper::NameDecoder::decodeClassName(typeid(CHILD)) << " with operation2 " << sofa::helper::NameDecoder::decodeClassName(typeid(ELMT1)) << "," << sofa::helper::NameDecoder::decodeClassName(typeid(ELMT2)) << " with return " << sofa::helper::NameDecoder::decodeClassName(typeid(RES_TYPE)) << " already in the factory" << std::endl;
            return 1;
        }

        getSingleton()->m_map[key] = (void*) f;

        return 0;
    }

    virtual DEFAULT_RETURN defaultFunc(PARAMS...) const = 0;

private:
    std::map<KEY_MAP,void*> m_map;

    static GenericOperation2 * getSingleton() {
        static GenericOperation2 * m_singleton = NULL;
        if (m_singleton == NULL) m_singleton = new CHILD();
        return m_singleton;
    }

    template<class RESTYPE>
    static RESTYPE t_defaultFunc(PARAMS... params) {
        return getSingleton()->defaultFunc(params...);
    }

    //No construction of the is is allowed
    GenericOperation2() {}
};

}

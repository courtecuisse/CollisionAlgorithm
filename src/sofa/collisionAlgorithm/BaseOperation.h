#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm::Operations {

class BaseOperation {
protected:
    BaseOperation() {}
};

template<class FUNC>
class GenericOperation {
public:

    static FUNC func(const BaseOperation * op) {
        auto it = m_map.find(op);
        if (it == m_map.end()) return m_default;
        return it->second;
    }

    static int register_func(const BaseOperation* op, FUNC f) {
        auto it = m_map.find(op);
        if (it != m_map.end()) std::cerr << "createCenterPointProximity with operation " << typeid(op).name() << " already in the factory" << std::endl;
        m_map[op] = f;
        return 0;
    }

private:
    static std::map<const BaseOperation*,FUNC> m_map;
    static const FUNC m_default;

};

class CreateCenterProximity : public GenericOperation<std::function<BaseProximity::SPtr(BaseElement::SPtr)>> {};

class Project : public GenericOperation<std::function<BaseProximity::SPtr(BaseProximity::SPtr, BaseElement::SPtr)>> {};

}

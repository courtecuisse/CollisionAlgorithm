#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class BaseOperations {
public:

    typedef std::function<BaseProximity::SPtr(BaseElement::SPtr)> CreateCenterProximityFunc;
    typedef std::function<BaseProximity::SPtr(BaseProximity::SPtr, BaseElement::SPtr)> ProjectFunc;

    static CreateCenterProximityFunc createCenterProximity(const BaseOperations * op) {
        auto it = m_createCenterProximity.find(op);
        if (it == m_createCenterProximity.end()) return &default_createCenterProximity;
        return it->second;
    }

    static ProjectFunc project(const BaseOperations * op) {
        auto it = m_project.find(op);
        if (it == m_project.end()) return &default_project;
        return it->second;
    }

private:
    static std::map<const BaseOperations*,CreateCenterProximityFunc> m_createCenterProximity;
    static std::map<const BaseOperations*,ProjectFunc> m_project;

protected :
    BaseOperations() {}

    static BaseProximity::SPtr default_createCenterProximity(BaseElement::SPtr ) {
        std::cerr << "createCenterPointProximity with operation not in the factory" << std::endl;
        return NULL;
    }

    static int register_createCenterProximity(const BaseOperations* op, CreateCenterProximityFunc f) {
        auto it = m_createCenterProximity.find(op);
        if (it != m_createCenterProximity.end()) std::cerr << "createCenterPointProximity with operation " << typeid(op).name() << " already in the factory" << std::endl;
        m_createCenterProximity[op] = f;
        return 0;
    }



    static BaseProximity::SPtr default_project(BaseProximity::SPtr, BaseElement::SPtr) {
        std::cerr << "project with operation not in the factory" << std::endl;
        return NULL;
    }

    static int register_project(const BaseOperations* op, ProjectFunc f) {
        auto it = m_project.find(op);
        if (it != m_project.end()) std::cerr << "project with operation " << typeid(op).name() << " already in the factory" << std::endl;
        m_project[op] = f;
        return 0;
    }

};

}

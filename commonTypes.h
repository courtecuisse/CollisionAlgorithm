#pragma once

#include <memory>
#include <map>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>
#include <SofaBaseTopology/TriangleSetTopologyContainer.h>
#include <sofa/core/ObjectFactory.h>

namespace collisionAlgorithm {

typedef sofa::defaulttype::Vector3 Vector3;
typedef sofa::core::topology::BaseMeshTopology::Triangle TTriangle;
typedef sofa::core::topology::BaseMeshTopology::Edge TEdge;
typedef sofa::core::objectmodel::BaseObject BaseObject;

class BaseMatrix {

};

template<class T>
class Data : public sofa::core::objectmodel::Data<T>{
public:

    Data(const char * name, T v,BaseObject * o)
    : sofa::core::objectmodel::Data<T>( o->initData(this, v, name, "Default Description")) {}

};

typedef enum {
    OPTIONAL = 0,
    REQUIRED = 1,
    UNIQUE = 2
} PORT_POLICY;

template<class T, PORT_POLICY = OPTIONAL>
class PortIn {
public:
    sofa::core::objectmodel::Data<std::string> d_name;

    PortIn(sofa::core::objectmodel::BaseObject * o)
    : d_name(o->initData(&d_name, std::string("IN_").append(T::getObjectType()).c_str(), "Port out")) {
        m_container = o;
    }

    PortIn(BaseObject * o,const char * name)
    : d_name(o->initData(&d_name, std::string("IN_").append(name).c_str(), "Port out")) {
        m_container = o;
    }

    T * operator->() {
        T* res;
        m_container->getContext()->get(res,d_name.getValue());
        if (res == NULL) std::cerr << "CANNOT FIN OUT PORT " << d_name.getValue() << std::endl;
        return res;
    }

    BaseObject * m_container;
};

template<class T, PORT_POLICY = OPTIONAL>
class PortOut {
public:
    sofa::core::objectmodel::Data<std::string> d_name;

    PortOut(sofa::core::objectmodel::BaseObject * o)
    : d_name(o->initData(&d_name, std::string("OUT_").append(T::getObjectType()).c_str(), "Port out")) {
        m_container = o;
    }

    PortOut(BaseObject * o,const char * name)
    : d_name(o->initData(&d_name, std::string("OUT_").append(name).c_str(), "Port out")) {
        m_container = o;
    }

    T * operator->() {
        T* res;
        m_container->getContext()->get(res,d_name.getValue());
        if (res == NULL) std::cerr << "CANNOT FIN OUT PORT " << d_name.getValue() << std::endl;
        return res;
    }

    BaseObject * m_container;
};

class DisplayFlag {
public:
    enum  DisplayMode {
        STATS,
        VISUAL,
        STATE,
        FORCEFIELD,
        MAPPING,
        COLLISION,
        WIREFRAME,
        Nflag
    };

    bool isActive(DisplayMode /*m*/) {
        return true;
    }
};

class VecID {
public:
    enum VecID_d {
        restPosition,
        position,
        freePosition,
        velocity,
        force,
        vec_x,
        vec_q,
        vec_d,
        vec_r,
        NVecID
    };
};

typedef VecID::VecID_d TVecId;

class State : public sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> {
public:

    std::vector<Vector3> & get(TVecId v) {
        sofa::core::VecCoordId vid;

        if (v == TVecId::restPosition) vid = sofa::core::VecCoordId::restPosition();
        else if (v == TVecId::position) vid = sofa::core::VecCoordId::position();


        sofa::helper::WriteAccessor<sofa::core::objectmodel::Data <VecCoord> > x = *this->write(vid);
        return x.wref();
    }

    static std::string getObjectType() {
        return "State";
    }
};

class Topology : public sofa::core::topology::BaseMeshTopology {
public:
    PortIn<State,REQUIRED> p_state;

    Topology()
    : sofa::core::topology::BaseMeshTopology()
    , p_state(this){}

    static std::string getObjectType() {
        return "Topology";
    }
};

class ForceField : public BaseObject {
public:

    PortOut<State,REQUIRED> p_state;

    ForceField()
    : p_state(this) {}

    static std::string getObjectType() {
        return "ForceField";
    }
};

#define DECLARE_CLASS(CLASS_NAME) \
SOFA_DECL_CLASS(CLASS_NAME) \
int declare_init##CLASS_NAME = sofa::core::RegisterObject("Description").add< CLASS_NAME >();

}

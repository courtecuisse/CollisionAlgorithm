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
#include <sofa/core/visual/VisualParams.h>

namespace collisionAlgorithm {

typedef sofa::defaulttype::Vector3 Vector3;
typedef sofa::defaulttype::Vec3i Vec3i;
typedef sofa::core::topology::BaseMeshTopology::Triangle TTriangle;
typedef sofa::core::topology::BaseMeshTopology::Edge TEdge;
typedef sofa::core::objectmodel::BaseObject BaseObject;
typedef sofa::core::visual::VisualParams VisualParams;
typedef sofa::core::topology::TopologyContainer Topology;
typedef sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3dTypes> State;
typedef sofa::defaulttype::BoundingBox BoundingBox;

class BaseMatrix {};

typedef enum {
    OPTIONAL = 0,
    REQUIRED = 1,
    UNIQUE = 2
} PORT_POLICY;

typedef sofa::core::VecCoordId VecCoordId;
typedef sofa::core::VecCoordId VecID;


template<class T>
using ReadAccessor = sofa::helper::ReadAccessor<sofa::core::objectmodel::Data<sofa::helper::vector<T > > >;
//helper::ReadAccessor< Data< sofa::helper::vector<Triangle> > > m_triangle = d_triangle;
//typedef sofa::helper::ReadAccessor< sofa::core::objectmodel::Data< sofa::helper::vector<Vector3> > > ReadAccessor;

template<class T>
class Data : public sofa::core::objectmodel::Data<T>{
public:

    Data(const char * name, T v,BaseObject * o)
    : sofa::core::objectmodel::Data<T>( o->initData(this, v, name, "Default Description")) {}

};

template<class T>
class InternalType {
public:
    T* search(sofa::core::objectmodel::BaseContext * ctx,const std::string & name) {
        T * m_object;
        ctx->get(m_object,name);
        if (m_object == NULL) std::cerr << "CANNOT FIND OUT PORT " << name << std::endl;
        return m_object;
    }

    typedef T OUT;
};

template<class T, PORT_POLICY = OPTIONAL>
class PortIn {
public:
    typedef typename InternalType<T>::OUT OUT;
    sofa::core::objectmodel::Data<std::string> d_name;

    PortIn(sofa::core::objectmodel::BaseObject * o)
    : d_name(o->initData(&d_name, std::string("IN_").append(typeid(o).name()).c_str(), "Port out")) {
        m_container = o;
    }

    PortIn(BaseObject * o,const char * name)
    : d_name(o->initData(&d_name, std::string("IN_").append(name).c_str(), "Port out")) {
        m_container = o;
    }

    OUT * operator->() {
        return m_convertor.search(m_container->getContext(),d_name.getValue());
    }

    OUT * operator()() {
        return m_convertor.search(m_container->getContext(),d_name.getValue());
    }

    InternalType<T> m_convertor;
    BaseObject * m_container;
};

template<class T, PORT_POLICY = OPTIONAL>
class PortOut {
public:
    typedef typename InternalType<T>::OUT OUT;
    sofa::core::objectmodel::Data<std::string> d_name;

    PortOut(sofa::core::objectmodel::BaseObject * o)
    : d_name(o->initData(&d_name, std::string("OUT_").append(typeid(o).name()).c_str(), "Port out")) {
        m_container = o;
    }

    PortOut(BaseObject * o,const char * name)
    : d_name(o->initData(&d_name, std::string("OUT_").append(name).c_str(), "Port out")) {
        m_container = o;
    }

    OUT * operator->() {
        return m_convertor.search(m_container->getContext(),d_name.getValue());
    }

    OUT * operator()() {
        return m_convertor.search(m_container->getContext(),d_name.getValue());
    }

    InternalType<T> m_convertor;
    BaseObject * m_container;
};

//template<>
//class InternalType<State> {
//public:
//    class Wrapper {
//    public:
//        Wrapper(State * obj) {
//            m_object = obj;
//        }

//        const std::string & getName() {
//            return m_object->getName();
//        }

//        static std::string getObjectType() {
//            return "State";
//        }

//        void computeBBox(BoundingBox & b) {
//            m_object->computeBBox(sofa::core::ExecParams::defaultInstance());

//            b = m_object->f_bbox.getValue();
//        }

//        ReadAccessor<Vector3> & read(TVecId v) {
//            sofa::core::VecCoordId vid;

//            if (v == TVecId::restPosition) vid = sofa::core::VecCoordId::restPosition();
//            else if (v == TVecId::position) vid = sofa::core::VecCoordId::position();


//            sofa::helper::WriteAccessor<sofa::core::objectmodel::Data <State::VecCoord> > x = *m_object->write(vid);
//            return x.wref();
//        }

//        State * m_object;
//    };

//    InternalType() {
//        m_wrapper = NULL;
//    }

//    ~InternalType() {
//        if (m_wrapper) delete m_wrapper;
//    }

//    Wrapper* search(sofa::core::objectmodel::BaseContext * ctx,const std::string & name) {
//        if (m_wrapper == NULL) {
//            State * m_object;
//            ctx->get(m_object,name);
//            if (m_object == NULL) std::cerr << "CANNOT FIND OUT PORT " << name << std::endl;
//            else m_wrapper = new Wrapper(m_object);
//        }
//        return m_wrapper;
//    }

//    Wrapper * m_wrapper;
//    typedef Wrapper OUT;
//};

template<>
class InternalType<Topology> {
public:
    class Wrapper {
    public:
        PortIn<State,REQUIRED> p_state;

        Wrapper(Topology * obj)
        : p_state(obj) {
            m_object = obj;
        }

        const std::string & getName() {
            return m_object->getName();
        }

        static std::string getObjectType() {
            return "Topology";
        }

        unsigned getNbPoints() {
            return m_object->getNbPoints();
        }

        unsigned getNbEdges() {
            return m_object->getNbEdges();
        }

        const Topology::SeqEdges & getEdges() {
            return m_object->getEdges();
        }

        TEdge getEdge(unsigned eid) {
            return m_object->getEdge(eid);
        }

        unsigned getNbTriangles() {
            return m_object->getNbTriangles();
        }

        const Topology::SeqTriangles & getTriangles() {
            return m_object->getTriangles();
        }

        TTriangle getTriangle(unsigned tid) {
            return m_object->getTriangle(tid);
        }

        Topology::TrianglesAroundVertex getTrianglesAroundVertex(unsigned pid) {
            return m_object->getTrianglesAroundVertex(pid);
        }

        Topology * m_object;
    };

    InternalType() {
        m_wrapper = NULL;
    }

    ~InternalType() {
        if (m_wrapper) delete m_wrapper;
    }

    Wrapper* search(sofa::core::objectmodel::BaseContext * ctx,const std::string & name) {
        if (m_wrapper == NULL) {
            Topology * m_object;
            ctx->get(m_object,name);
            if (m_object == NULL) std::cerr << "CANNOT FIND OUT PORT " << name << std::endl;
            else m_wrapper = new Wrapper(m_object);
        }
        return m_wrapper;
    }

    Wrapper * m_wrapper;
    typedef Wrapper OUT;
};

//template<>
//class InternalType<BaseObject> {
//public:
//    class Wrapper {
//    public:
//        Wrapper(BaseObject * obj) {
//            m_object = obj;
//        }

//        static std::string getObjectType() {
//            return "BaseObject";
//        }

//        BaseObject * m_object;
//    };

//    InternalType() {
//        m_wrapper = NULL;
//    }

//    ~InternalType() {
//        if (m_wrapper) delete m_wrapper;
//    }

//    Wrapper* search(sofa::core::objectmodel::BaseContext * ctx,const std::string & name) {
//        if (m_wrapper == NULL) {
//            BaseObject * m_object;
//            ctx->get(m_object,name);
//            if (m_object == NULL) std::cerr << "CANNOT FIND OUT PORT " << name << std::endl;
//            else m_wrapper = new Wrapper(m_object);
//        }
//        return m_wrapper;
//    }

//    Wrapper * m_wrapper;
//    typedef Wrapper OUT;
//};

#define DECLARE_CLASS(CLASS_NAME) \
SOFA_DECL_CLASS(CLASS_NAME) \
int declare_init##CLASS_NAME = sofa::core::RegisterObject("Description").add< CLASS_NAME >();

}

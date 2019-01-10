#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

//Internal iterator of elements
template<class DataTypes>
class EdgeElementIterator : public ElementIterator {
public:
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    EdgeElementIterator(const EdgeGeometry<DataTypes> * geo) : m_geometry(geo) {
        m_state = m_geometry->l_state.get();
    }

    BaseProximity::SPtr project(const defaulttype::Vector3 & /*P*/) const {
        const core::topology::BaseMeshTopology::Edge edge = m_geometry->d_edges.getValue()[id()];
        return BaseProximity::SPtr(new EdgeProximity<DataTypes>(edge[0],edge[1],0.5,0.5,m_state));
    }

    virtual BaseProximity::SPtr center() const {
        const core::topology::BaseMeshTopology::Edge edge = m_geometry->d_edges.getValue()[id()];
        return BaseProximity::SPtr(new EdgeProximity<DataTypes>(edge[0],edge[1],0.5,0.5,m_state));
    }

    const EdgeGeometry<DataTypes> * m_geometry;
    State * m_state;
    unsigned m_size;
};


//    void draw(const core::visual::VisualParams * /*vparams*/) const override
//    {
//        glColor4dv(geometry()->d_color.getValue().data());

//        glBegin(GL_LINES);
//            glVertex3dv(getControlPoint(0)->getPosition().data());
//            glVertex3dv(getControlPoint(1)->getPosition().data());
//        glEnd();
//    }

//protected:
//    const EdgeGeometry* m_geometry;
//    size_t m_pid[2];
//    size_t m_eid;
//};

}

}

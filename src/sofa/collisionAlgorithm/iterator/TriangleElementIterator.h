#pragma once

#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class TriangleElementIterator : public DefaultElement {
public:
    typedef typename TriangleGeometry<DataTypes>::TriangleInfo TriangleInfo;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;

    TriangleElementIterator(const TriangleGeometry<DataTypes> * geo)
    : m_geometry(geo) {
        m_state = m_geometry->l_state.get();
    }

    virtual BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {
        core::topology::BaseMeshTopology::Triangle triangle;
        defaulttype::Vector3 factor;
        m_geometry->project(id(),P, triangle, factor);

        return BaseProximity::SPtr(new TriangleProximity<DataTypes>(id(), triangle[0],triangle[1],triangle[2], factor[0],factor[1],factor[2],m_geometry,m_state));
    }

    virtual BaseProximity::SPtr center() const {
        const core::topology::BaseMeshTopology::Triangle & triangle = m_geometry->d_triangles.getValue()[id()];
        return BaseProximity::SPtr(new TriangleProximity<DataTypes>(id(), triangle[0],triangle[1],triangle[2],0.3333,0.3333,0.3333,m_geometry,m_state));
    }

    bool end(const BaseGeometry * /*geo*/) const {
        return id() >= m_geometry->d_triangles.getValue().size();
    }

    virtual defaulttype::BoundingBox getBBox() const {
        const core::topology::BaseMeshTopology::Triangle & triangle = m_geometry->d_triangles.getValue()[id()];
        const helper::ReadAccessor<Data <VecCoord> >& x = *m_state->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[triangle[0]]);
        bbox.include(x[triangle[1]]);
        bbox.include(x[triangle[2]]);
        return bbox;
    }

    const TriangleGeometry<DataTypes> * m_geometry;
    State * m_state;
};


//    void drawTriangle(const core::visual::VisualParams * /*vparams*/,const defaulttype::Vector3 & A,const defaulttype::Vector3 & B, const defaulttype::Vector3 & C) const
//    {
//        double delta = 0.1;
//        defaulttype::Vector4 color = geometry()->d_color.getValue();

//        glBegin(GL_TRIANGLES);
//            glColor4f(fabs(color[0]-delta),color[1],color[2],color[3]);
//            glVertex3dv(A.data());
//            glColor4f(color[0],fabs(color[1]-delta),color[2],color[3]);
//            glVertex3dv(B.data()); // A<->B
//            glColor4f(color[0],color[1],fabs(color[2]-delta),color[3]);
//            glVertex3dv(C.data());
//        glEnd();
//    }

//    virtual void draw(const core::visual::VisualParams *vparams) const override
//    {
//        drawTriangle(vparams,getControlPoint(0)->getPosition(),
//                             getControlPoint(1)->getPosition(),
//                             getControlPoint(2)->getPosition());
//    }

//protected:
//    const TriangleGeometry* m_geometry;
//    size_t m_pid[3];
//    size_t m_eid;
//};

}

}

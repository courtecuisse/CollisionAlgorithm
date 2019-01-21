#pragma once

#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/proximity/TriangleProximity.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
void TriangleGeometry<DataTypes>::prepareDetection()
{
    d_triangles.prepareDetection();
}

template<class DataTypes>
void TriangleGeometry<DataTypes>::init()
{
    ///To remove if we think every input has to be explicit
    if(d_triangles.getValue().empty())
    {
        msg_warning(this) << "Triangles are not set (data is empty). Will set from topology if present in the same context";
        sofa::core::topology::BaseMeshTopology* topology{nullptr};
        this->getContext()->get(topology);
        if(!topology)
        {
            msg_error(this) << "No topology to work with ; giving up.";
        }
        else
        {
            if(topology->getTriangles().empty())
            {
                msg_error(this) << "No topology with triangles to work with ; giving up.";
            }
            else
            {
                d_triangles.setParent(topology->findData("triangles"));
            }
        }
    }
}

template<class DataTypes>
void TriangleGeometry<DataTypes>::draw(const core::visual::VisualParams * vparams) {
    if (! vparams->displayFlags().getShowCollisionModels())
        return;

    if (this->d_color.getValue()[3] == 0.0)
        return;

    glDisable(GL_LIGHTING);

    double delta = 0.1;
    defaulttype::Vector4 color = this->d_color.getValue();
    const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());

    glBegin(GL_TRIANGLES);
    for (unsigned i=0;i<d_triangles.getValue().size();i++) {
        const Triangle& tri = this->d_triangles.getValue()[i];

        glColor4f(fabs(color[0]-delta),color[1],color[2],color[3]);
        glVertex3dv(pos[tri[0]].data());
        glColor4f(color[0],fabs(color[1]-delta),color[2],color[3]);
        glVertex3dv(pos[tri[1]].data());
        glColor4f(color[0],color[1],fabs(color[2]-delta),color[3]);
        glVertex3dv(pos[tri[2]].data());
    }
    glEnd();
}

}

}

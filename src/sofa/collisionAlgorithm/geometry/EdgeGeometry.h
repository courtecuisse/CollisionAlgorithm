#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class EdgeGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef EdgeGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;
    typedef sofa::core::topology::BaseMeshTopology::Edge Edge;
    typedef helper::vector<Edge> VecEdges;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<VecEdges> d_edges;

    EdgeGeometry()
    : d_edges(initData(&d_edges, "edges", "Vector of Edges")){}

    virtual BaseElementIterator::UPtr getElementIterator(unsigned eid = 0) const {
        return DefaultElementIterator<EdgeElement<GEOMETRY> >::create(this, d_edges.getValue().size(), eid);
    }

    inline const Edge & getEdge(unsigned eid) const {
        return d_edges.getValue()[eid];
    }

    virtual void draw(const core::visual::VisualParams * vparams) {
        Inherit::draw(vparams);

        if (! vparams->displayFlags().getShowCollisionModels())
            return;

        if (this->d_color.getValue()[3] == 0.0)
            return;

        glDisable(GL_LIGHTING);

        glBegin(GL_LINES);
        glColor4dv(this->d_color.getValue().data());
        const helper::ReadAccessor<DataVecCoord> & pos = this->l_state->read(core::VecCoordId::position());

        for (unsigned i=0;i<d_edges.getValue().size();i++) {
            const Edge & edge = this->d_edges.getValue()[i];

            glVertex3dv(pos[edge[0]].data());
            glVertex3dv(pos[edge[1]].data());
        }
        glEnd();
    }
};

}

}

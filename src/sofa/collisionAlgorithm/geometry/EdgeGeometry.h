#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/elements/DataEdgeElement.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class EdgeGeometry : public PointGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef PointGeometry<DataTypes> Inherit;
    typedef EdgeGeometry<DataTypes> GEOMETRY;

    SOFA_CLASS(GEOMETRY,Inherit);

    DataEdgeContainer<GEOMETRY> d_edges;

    EdgeGeometry()
    : d_edges(initData(&d_edges,"edges", "Edges Container" )) {}

    void init() {
        Inherit::init();

        ///To remove if we think every input has to be explicit
        if(d_edges.getValue().empty())
        {
            msg_warning(this) << "Edges are not set (data is empty). Will set from topology if present in the same context";
            sofa::core::topology::BaseMeshTopology* topology{nullptr};
            this->getContext()->get(topology);
            if(!topology)
            {
                msg_error(this) << "No topology to work with ; giving up.";
            }
            else
            {
                if(topology->getEdges().empty())
                {
                    msg_error(this) << "No topology with edges to work with ; giving up.";
                }
                else
                {
                    d_edges.setParent(topology->findData("edges"));
                }
            }
        }
    }
};

}

}

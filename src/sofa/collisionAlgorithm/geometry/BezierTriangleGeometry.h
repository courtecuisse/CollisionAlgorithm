#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/collisionAlgorithm/elements/DataBezierTriangleElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class BezierTriangleGeometry : public EdgeGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef EdgeGeometry<DataTypes> Inherit;
    typedef BezierTriangleGeometry<DataTypes> GEOMETRY;

    SOFA_CLASS(GEOMETRY,Inherit);

    DataBezierTriangleContainer<GEOMETRY> d_triangles;

    BezierTriangleGeometry()
    : d_triangles(initData(&d_triangles,"triangles", "Triangles Container" )) {}

    void init() {
        Inherit::init();

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

};

}

}

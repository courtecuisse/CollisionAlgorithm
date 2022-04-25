#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa::collisionAlgorithm {


template<class DataTypes>
class VectorPointNormalHandler : public CollisionComponent {
public:

    SOFA_CLASS(SOFA_TEMPLATE(VectorPointNormalHandler,DataTypes), CollisionComponent);

    Data<type::vector<type::Vector3>> d_normals;

    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef typename GEOMETRY::ELEMENT ELEMENT;

    core::objectmodel::SingleLink<VectorPointNormalHandler<DataTypes>,PointGeometry<DataTypes>,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    VectorPointNormalHandler()
    : d_normals(initData(&d_normals, "normals", "Vector of normals"))
    , l_geometry(initLink("geometry","Link to Geometry")){}


    void init() {

        for (unsigned i=0; i<d_normals.getValue().size(); i++)
        {
            l_geometry->getTopoProxIdx(i)->setNormal(d_normals.getValue()[i]);
        }

    }

    void prepareDetection() override {}

};


}

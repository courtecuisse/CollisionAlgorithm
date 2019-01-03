#pragma once

#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>
#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/core/topology/BaseMeshTopology.h>

namespace sofa
{

namespace collisionAlgorithm
{

class TriangleElement;

class TriangleGeometry : public BaseGeometry
{
    friend class TriangleElement;
    friend class TriangleProximity;

public:
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    //typedef sofa::core::topology::BaseMeshTopology::TriangleID TriangleID;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;

    SOFA_CLASS(TriangleGeometry,BaseGeometry);

    TriangleGeometry()
        : BaseGeometry()
        , d_triangles(initData(&d_triangles, VecTriangles(), "triangles", "Vector of Triangles"))
    {

    }

    virtual ~TriangleGeometry() override {}

    virtual void init() override;

    virtual void prepareDetection() override;

    ConstraintProximity::SPtr createProximity(const TriangleElement * elmt,double f1,double f2,double f3) const;

    inline const VecTriangles& triangles() const
    {
        return d_triangles.getValue();
    }

    typedef struct
    {
        defaulttype::Vector3 v0,v1;
        double d00;
        double d01;
        double d11;
        double invDenom;

        defaulttype::Vector3 tn,ax1,ax2;
    } TriangleInfo;

    inline const std::vector<defaulttype::Vector3>& pointNormals() const
    {
        return this->m_pointNormal;
    }

    inline const TriangleInfo& triangleInfo(size_t index) const
    {
        return this->m_triangle_info[index];
    }

    Data<VecTriangles> d_triangles;

protected:
    virtual void setNormalHandler() override
    {
        if(!l_normalHandler.get())
        {
            msg_warning(this) << "No normal handler given, finding one...";
            l_normalHandler.setPath("@.");
        }
        if(!l_normalHandler.get())
        {
            msg_warning(this) << "No normal handler found, creating a flat one";
            SofaBaseNormalHandler::SPtr normalHandler = sofa::core::objectmodel::New<SofaFlatNormalHandler<TriangleGeometry> >(this);
            this->getContext()->addObject(normalHandler);
            l_normalHandler.set(normalHandler);
        }
    }

    std::vector<TriangleInfo> m_triangle_info;
    std::vector<defaulttype::Vector3> m_pointNormal;
    std::vector< std::vector<TriangleID> > m_trianglesAroundVertex;
};


}

}

#pragma once

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PhongTriangleProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class DataTypes>
class PhongTriangleGeometry : public TriangleGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef TriangleGeometry<DataTypes> Inherit;
    typedef PhongTriangleGeometry<DataTypes> GEOMETRY;
    typedef typename DataTypes::Coord Coord;
    typedef Data<helper::vector<defaulttype::Vector3> > DataVecCoord;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;

    SOFA_CLASS(GEOMETRY,Inherit);

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const {
        return DefaultElementIterator<GEOMETRY >::create(this, this->d_triangles.getValue().size(), eid);
    }

    inline BaseProximity::SPtr project(unsigned tid, const defaulttype::Vector3 & P) const {
        core::topology::BaseMeshTopology::Triangle triangle;
        defaulttype::Vector3 factor;
        TriangleGeometry<DataTypes>::project(tid, P, triangle, factor);

        return BaseProximity::create<PhongTriangleProximity<GEOMETRY> >(this,tid,
                                                                         triangle[0],triangle[1],triangle[2],
                                                                         factor[0],factor[1],factor[2]);
    }

    inline BaseProximity::SPtr center(unsigned tid) const {
        const core::topology::BaseMeshTopology::Triangle & triangle = this->d_triangles.getValue()[tid];
        return BaseProximity::create<PhongTriangleProximity<GEOMETRY> >(this,tid,
                                                                         triangle[0],triangle[1],triangle[2],
                                                                         0.3333,0.3333,0.3333);
    }

    virtual void init() {
        //store triangles around vertex information
        const VecTriangles& triangles = this->d_triangles.getValue();
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
        m_trianglesAroundVertex.resize(pos.size());
        for (size_t i = 0; i < triangles.size(); ++i)
        {
            // adding edge i in the edge shell of both points
            for (size_t j=0; j<3; ++j)
                m_trianglesAroundVertex[triangles[i][j]].push_back(i);
        }
    }

    virtual void computeCollisionReset() {
        TriangleGeometry<DataTypes>::computeCollisionReset();

        m_point_normals.resize(m_trianglesAroundVertex.size());

        for (size_t p=0;p<m_trianglesAroundVertex.size();p++)
        {
            const std::vector<TriangleID> & tav = m_trianglesAroundVertex[p];
            m_point_normals[p] = defaulttype::Vector3(0,0,0);
            for (size_t t=0;t<tav.size();t++)
            {
                m_point_normals[p] += this->m_triangle_normals[tav[t]];
            }
            m_point_normals[p].normalize();
        }
    }

    inline const helper::vector<defaulttype::Vector3> & pointNormals() const {
        return m_point_normals;
    }

protected:
    std::vector<defaulttype::Vector3> m_point_normals;
    std::vector< std::vector<TriangleID> > m_trianglesAroundVertex;
};


}

}

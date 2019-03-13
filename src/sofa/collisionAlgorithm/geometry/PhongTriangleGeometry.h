#pragma once

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PhongTriangleProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class CONTAINER>
class PhongTriangleElement : public BaseElement {
public:
    typedef CONTAINER TContainer;
    typedef typename CONTAINER::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef Data<VecCoord> DataVecCoord;

    PhongTriangleElement(unsigned id,const CONTAINER * geo) : m_tid(id), m_geo(geo) {}

    inline BaseProximity::SPtr project(const defaulttype::Vector3 & P) const {
        core::topology::BaseMeshTopology::Triangle triangle;
        defaulttype::Vector3 factor;
        m_geo->project(m_tid, P, triangle, factor);

        return BaseProximity::create<PhongTriangleProximity<DataTypes> >(m_geo->getState(),
                                                                         triangle[0],triangle[1],triangle[2],
                                                                         factor[0],factor[1],factor[2],
                                                                         m_geo->pointNormals()[triangle[0]],
                                                                         m_geo->pointNormals()[triangle[1]],
                                                                         m_geo->pointNormals()[triangle[2]]);
    }

    inline BaseProximity::SPtr center() const {
        const core::topology::BaseMeshTopology::Triangle & triangle = m_geo->getTriangles()[m_tid];
        return BaseProximity::create<PhongTriangleProximity<DataTypes> >(m_geo->getState(),
                                                                         triangle[0],triangle[1],triangle[2],
                                                                         0.3333,0.3333,0.3333,
                                                                         m_geo->pointNormals()[triangle[0]],
                                                                         m_geo->pointNormals()[triangle[1]],
                                                                         m_geo->pointNormals()[triangle[2]]);
    }

    inline defaulttype::BoundingBox getBBox() const {
        const core::topology::BaseMeshTopology::Triangle & triangle = m_geo->getTriangles()[m_tid];
        const helper::ReadAccessor<Data <VecCoord> >& x = m_geo->getState()->read(core::VecCoordId::position());
        defaulttype::BoundingBox bbox;
        bbox.include(x[triangle[0]]);
        bbox.include(x[triangle[1]]);
        bbox.include(x[triangle[2]]);
        return bbox;
    }

protected:
    unsigned m_tid;
    const CONTAINER * m_geo;
};

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
        return DefaultElementIterator<PhongTriangleElement<GEOMETRY> >::create(this, this->d_triangles.getValue().size(), eid);
    }

    virtual void init() {
        Inherit::init();

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

    virtual void prepareDetection() {
        m_point_normals.resize(m_trianglesAroundVertex.size());

        for (size_t p=0;p<m_trianglesAroundVertex.size();p++)
        {
            const std::vector<TriangleID> & tav = m_trianglesAroundVertex[p];
            m_point_normals[p] = defaulttype::Vector3(0,0,0);
            for (size_t t=0;t<tav.size();t++) {
                m_point_normals[p] += this->triangleNormals()[tav[t]];
            }
            m_point_normals[p].normalize();
        }
    }

    inline const helper::vector<defaulttype::Vector3> & pointNormals() const {
        return m_point_normals;
    }

protected:
    helper::vector<defaulttype::Vector3> m_point_normals;
    helper::vector< std::vector<TriangleID> > m_trianglesAroundVertex;
};


}

}

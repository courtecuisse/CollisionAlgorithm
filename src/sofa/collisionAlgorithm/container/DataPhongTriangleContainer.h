#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/PhongTriangleProximity.h>
#include <sofa/collisionAlgorithm/elements/DataTriangleElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY>
class DataPhongTriangleElement : public DataTriangleElement<GEOMETRY> {
public:

    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;
    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef sofa::core::topology::BaseMeshTopology::Triangle Triangle;
    typedef size_t TriangleID; // to remove once TriangleID has been changed to size_t in BaseMeshTopology
    typedef helper::vector<Triangle> VecTriangles;

    explicit DataPhongTriangleElement(const typename DataTriangleElement<GEOMETRY>::InitData& init)
    : DataTriangleElement<GEOMETRY>(init) {}

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) const {
        return DefaultElementIterator<DataPhongTriangleElement<GEOMETRY> >::create(this, this->getValue().size(), eid);
    }

    inline BaseProximity::SPtr project(unsigned tid, const defaulttype::Vector3 & P) const {
        core::topology::BaseMeshTopology::Triangle triangle;
        defaulttype::Vector3 factor;
        DataTriangleElement<GEOMETRY>::project(tid, P, triangle, factor);

        return BaseProximity::create<PhongTriangleProximity<DataTypes> >(this->m_geometry->getState(),
                                                                         triangle[0],triangle[1],triangle[2],
                                                                         factor[0],factor[1],factor[2],
                                                                         m_point_normals[triangle[0]],m_point_normals[triangle[1]],m_point_normals[triangle[2]]);
    }

    inline BaseProximity::SPtr center(unsigned tid) const {
        const core::topology::BaseMeshTopology::Triangle & triangle = this->getValue()[tid];
        return BaseProximity::create<PhongTriangleProximity<DataTypes> >(this->m_geometry->getState(),
                                                                         triangle[0],triangle[1],triangle[2],
                                                                         0.3333,0.3333,0.3333,
                                                                         m_point_normals[triangle[0]],m_point_normals[triangle[1]],m_point_normals[triangle[2]]);
    }


    virtual void init() {
        //store triangles around vertex information
        const VecTriangles& triangles = this->getValue();
        const helper::ReadAccessor<DataVecCoord> & pos = this->m_geometry->getState()->read(core::VecCoordId::position());
        m_trianglesAroundVertex.resize(pos.size());
        for (size_t i = 0; i < triangles.size(); ++i)
        {
            // adding edge i in the edge shell of both points
            for (size_t j=0; j<3; ++j)
                m_trianglesAroundVertex[triangles[i][j]].push_back(i);
        }
    }

    virtual void prepareDetection() {
        DataTriangleElement<GEOMETRY>::prepareDetection();

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

protected:
    std::vector<defaulttype::Vector3> m_point_normals;
    std::vector< std::vector<TriangleID> > m_trianglesAroundVertex;
};


}

}

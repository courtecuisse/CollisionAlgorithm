#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class EdgeNormalHandler : public CollisionComponent {
public:

    SOFA_CLASS(SOFA_TEMPLATE(EdgeNormalHandler,DataTypes), CollisionComponent);

    typedef EdgeGeometry<DataTypes> GEOMETRY;
    typedef typename GEOMETRY::ELEMENT ELEMENT;

    core::objectmodel::SingleLink<EdgeNormalHandler<DataTypes>,EdgeGeometry<DataTypes>,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    EdgeNormalHandler()
    : l_geometry(initLink("geometry","Link to TriangleGeometry")){}

    class LinkEdgeProximity : public DefaultEdgeProximity<DataTypes> {
    public:
        typedef sofa::core::behavior::MechanicalState<DataTypes> State;
        typedef typename DataTypes::VecCoord VecCoord;
        typedef core::objectmodel::Data< VecCoord >        DataVecCoord;

        LinkEdgeProximity(State * s, unsigned p0,unsigned p1,double f0,double f1)
        : DefaultEdgeProximity<DataTypes>(s,p0,p1,f0,f1) {}

        sofa::type::Vector3 getNormal() const override {
            const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
            return (pos[this->m_p1] - pos[this->m_p0]).normalized();
        }

    };


    void init() {
        l_geometry->setCreateProximity(
        [=](const EdgeElement * elmt, double f0,double f1) -> BaseProximity::SPtr {
                return BaseProximity::create<LinkEdgeProximity>(l_geometry->getState(),
                                                                      elmt->getP0(),elmt->getP1(),
                                                                      f0,f1);
            }
        );
    }

    void prepareDetection() override {}

};

}

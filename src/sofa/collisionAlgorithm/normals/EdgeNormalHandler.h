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

    class LinkEdgeProximity : public EdgeProximity {
    public:
        typedef sofa::core::behavior::MechanicalState<DataTypes> State;
        typedef typename DataTypes::VecCoord VecCoord;
        typedef core::objectmodel::Data< VecCoord >        DataVecCoord;

        LinkEdgeProximity(BaseProximity::SPtr p0,BaseProximity::SPtr p1,double f0,double f1)
        : EdgeProximity(p0,p1,f0,f1) {}

        sofa::type::Vector3 getNormal() const override {
            return ((this->m_p1->getPosition() - this->m_p0->getPosition()).normalized());
        }

    };


    void init() {
        l_geometry->setCreateEdgeProximity(
        [=](const EdgeElement * elmt, double f0,double f1) -> BaseProximity::SPtr {
                return BaseProximity::create<LinkEdgeProximity>(elmt->getP0(), elmt->getP1(), f0, f1);
            }
        );
    }

    void prepareDetection() override {}

};

}

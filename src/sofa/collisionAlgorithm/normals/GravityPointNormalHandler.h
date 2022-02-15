#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/geometry/PointGeometry.h>

namespace sofa::collisionAlgorithm {

template<class DataTypes>
class GravityPointNormalHandler : public CollisionComponent {
public:

    typedef sofa::core::behavior::MechanicalState<DataTypes> State;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef typename GEOMETRY::ELEMENT ELEMENT;

    SOFA_CLASS(SOFA_TEMPLATE(GravityPointNormalHandler,DataTypes), CollisionComponent);

    core::objectmodel::SingleLink<GravityPointNormalHandler<DataTypes>,GEOMETRY,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    GravityPointNormalHandler()
    : l_geometry(initLink("geometry","Link to TriangleGeometry")){}

    class GravityPointNormalProximity : public DefaultPointProximity<DataTypes> {
    public:
        typedef sofa::core::behavior::MechanicalState<DataTypes> State;
        typedef typename DataTypes::VecCoord VecCoord;
        typedef core::objectmodel::Data< VecCoord >        DataVecCoord;

        GravityPointNormalProximity(State * s, unsigned p0, const type::Vector3 & G)
        : DefaultPointProximity<DataTypes>(s,p0)
        , m_G(G){}

        sofa::type::Vector3 getNormal() const override {
            const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
            return (pos[this->m_pid] - m_G).normalized();
        }

    private:
        const type::Vector3 & m_G;

    };


    void init() {
        for (auto it = l_geometry->begin();it != l_geometry->end(); it++) {
            ELEMENT * elmt = it->element_cast();
            elmt->setProximityCreator(
                [=](const PointElement * elmt) -> BaseProximity::SPtr {
                    return BaseProximity::SPtr(new GravityPointNormalProximity(l_geometry->getState(),
                                                                               elmt->getP0(),
                                                                               m_gcenter));
            });
        }
    }

    void prepareDetection() override {
        m_gcenter = type::Vector3();

        const helper::ReadAccessor<DataVecCoord> & pos = this->l_geometry->getState()->read(core::VecCoordId::position());

        for (unsigned i=0;i<pos.size();i++) {
            m_gcenter += pos[i];
        }

        if (pos.size()) m_gcenter*=1.0/pos.size();
    }

private :
    type::Vector3 m_gcenter;
};

}

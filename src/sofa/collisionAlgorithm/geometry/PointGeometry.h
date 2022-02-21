#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/toolbox/PointToolBox.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class PointGeometry : public TBaseGeometry<DataTypes,PointElement>, public PointProximityCreator {
public:
    typedef DataTypes TDataTypes;
    typedef PointElement ELEMENT;
    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes,ELEMENT> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef std::function<BaseProximity::SPtr(const PointElement * elmt)> ProximityCreatorFunc;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<double> d_drawRadius;

    PointGeometry()
    : d_drawRadius(initData(&d_drawRadius, (double) 1.0, "drawRadius", "radius of drawing")) {
        f_createProximity = [=](const PointElement * elmt) -> BaseProximity::SPtr {
            return BaseProximity::SPtr(new DefaultPointProximity<DataTypes>(this->getState(),elmt->getP0()));
        };
    }

    type::Vector3 getPosition(unsigned pid) override {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
        return pos[pid];
    }

    void init() {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
        for (unsigned i=0;i<pos.size();i++) {
            m_elements.push_back(this->createElement(this,i));
        }
    }

    BaseProximity::SPtr createProximity(const PointElement * elmt) override {
        return f_createProximity(elmt);
    }

    void prepareDetection() override {}

    ElementIterator::SPtr begin() const override {
        return ElementIterator::SPtr(new TDefaultElementIterator(m_elements));
    }

    void setCreateProximity(ProximityCreatorFunc f) {
        f_createProximity = f;
    }

private:
    std::vector<PointElement::SPtr> m_elements;
    ProximityCreatorFunc f_createProximity;
};

}

}

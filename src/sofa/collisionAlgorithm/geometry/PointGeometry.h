#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/toolbox/PointToolBox.h>
#include <sofa/collisionAlgorithm/proximity/TopologyProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class PointGeometry : public TBaseGeometry<DataTypes>/*, public PointProximityCreator*/ {
public:
    typedef DataTypes TDataTypes;
    typedef PointElement ELEMENT;
    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef std::function<BaseProximity::SPtr(const PointElement * elmt)> ProximityCreatorFunc;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<double> d_drawRadius;

    PointGeometry()
    : d_drawRadius(initData(&d_drawRadius, (double) 1.0, "drawRadius", "radius of drawing")) {
//        f_createProximity = [=](const PointElement * elmt) -> BaseProximity::SPtr {
//            return BaseProximity::create<PointProximity>(elmt->getP0());
//        };
    }

//    type::Vector3 getPosition(unsigned pid) override {
//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
//        return pos[pid];
//    }

    void init() {
        for (unsigned j=0; j<this->getState()->getSize(); j++) {
            this->m_topoProx.push_back(TBaseProximity<DataTypes>::template create<TopologyProximity<DataTypes>>(this->getState(), j));
        }

        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
        for (unsigned i=0;i<pos.size();i++) {
            auto elmt = BaseElement::create<PointElement>(this->m_topoProx[i]);
            m_elements.push_back(elmt);
        }
        if (f_createProximity != NULL) setCreateProximity(f_createProximity);
    }

//    BaseProximity::SPtr createProximity(const PointElement * elmt) override {
//        return f_createProximity(elmt);
//    }

    void prepareDetection() override {}

    ElementIterator::SPtr begin() const override {
        return ElementIterator::SPtr(new TDefaultElementIteratorSPtr(m_elements));
    }

    void setCreateProximity(ProximityCreatorFunc f) {
        for (unsigned i=0; i< m_elements.size(); i++) {
            m_elements[i]->setCreateProximity(f);
        }
    }

    void setCreateProxFunc(ProximityCreatorFunc f) {
        f_createProximity = f;
    }

private:
    std::vector<PointElement::SPtr> m_elements;
    ProximityCreatorFunc f_createProximity;
};

}

}

#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/toolbox/PointToolBox.h>
#include <sofa/collisionAlgorithm/proximity/TopologyProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class PointGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef PointElement ELEMENT;
    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef std::function<BaseProximity::SPtr(const PointElement * elmt)> ProximityCreatorFunc;

    SOFA_CLASS(GEOMETRY,Inherit);


    PointGeometry() {}

//    type::Vector3 getPosition(unsigned pid) override {
//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
//        return pos[pid];
//    }

//    void init() {
////        this->m_topoProx.clear();
////        for (unsigned j=0; j<this->getState()->getSize(); j++) {
////            this->m_topoProx.push_back(TBaseProximity<DataTypes>::template create<TopologyProximity<DataTypes>>(this->getState(), j));
////        }

//        TBaseGeometry<DataTypes>::init();

//        m_pointElements.clear();
//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
//        for (unsigned i=0;i<pos.size();i++) {
//            auto elmt = BaseElement::create<PointElement>(this->m_topoProx[i]);
//            m_pointElements.push_back(elmt);
//        }
//    }

    void buildPointElements() override {
        m_pointElements.clear();
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
        for (unsigned i=0;i<pos.size();i++) {
            auto elmt = BaseElement::create<PointElement>(this->m_topoProx[i]);
            m_pointElements.push_back(elmt);
        }
    }


    void prepareDetection() override {}

    ElementIterator::SPtr begin(unsigned id = 0) const override {
        return pointBegin(id);
    }

    inline ElementIterator::SPtr pointBegin(unsigned id = 0) const {
        return ElementIterator::SPtr(new TDefaultElementIteratorSPtr(m_pointElements,id));
    }

    void setCreatePointProximity(ProximityCreatorFunc f) {
        for (unsigned i=0; i< m_pointElements.size(); i++) {
            m_pointElements[i]->setCreateProximity(f);
        }
    }

    inline std::vector<PointElement::SPtr> & getElements() {
        return m_pointElements;
    }


private:
    std::vector<PointElement::SPtr> m_pointElements;
};

}

}

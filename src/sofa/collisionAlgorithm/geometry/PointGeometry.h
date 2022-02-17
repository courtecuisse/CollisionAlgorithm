#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/toolbox/PointToolBox.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class PointGeometry : public TBaseGeometry<DataTypes,PointElement> {
public:
    typedef DataTypes TDataTypes;
    typedef PointElement ELEMENT;
    typedef PointGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes,ELEMENT> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<double> d_drawRadius;

    PointGeometry()
    : d_drawRadius(initData(&d_drawRadius, (double) 1.0, "drawRadius", "radius of drawing")) {}

    void init() {
        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

        //default proximity creator
        auto f = [=](const PointElement * elmt) -> BaseProximity::SPtr {
            return BaseProximity::SPtr(new DefaultPointProximity<DataTypes>(this->getState(),elmt->getP0()));
        };

        for (unsigned i=0;i<pos.size();i++) {
            PointElement::SPtr elmt = this->createElement(i,i,f);
            m_elements.push_back(elmt);
        }
    }

    void prepareDetection() override {}

    ElementIterator::SPtr begin() const override {
        return ElementIterator::defaultIterator(m_elements);
    }

//    void draw(const core::visual::VisualParams *vparams) override {
//        if(!(this->d_draw.getValue())) return;
////        this->drawNormals(vparams);
////        if (! vparams->displayFlags().getShowCollisionModels()) return;
//        if (! vparams->displayFlags().getShowCollisionModels()) {
//            return ;
//        }
//        const sofa::type::RGBAColor & color = this->d_color.getValue();
//        if (color[3] == 0.0) return;
//        if (d_drawRadius.getValue() == 0.0) return;

//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

//        glColor4f(color[0],color[1],color[2],color[3]);

//        for(auto it=this->begin();it != this->end();it++) {
//            vparams->drawTool()->drawSphere(pos[it->id()],d_drawRadius.getValue());
//        }
//    }

private:
    std::vector<PointElement::SPtr> m_elements;
};

}

}

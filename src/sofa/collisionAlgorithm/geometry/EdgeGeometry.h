#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/toolbox/EdgeToolBox.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class EdgeGeometry : public TBaseGeometry<DataTypes,EdgeElement> {
public:
    typedef DataTypes TDataTypes;
    typedef EdgeElement ELEMENT;
    typedef EdgeGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes,ELEMENT> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;

    SOFA_CLASS(GEOMETRY,Inherit);

    core::objectmodel::SingleLink<GEOMETRY,core::topology::BaseMeshTopology,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_topology;

    EdgeGeometry()
    : l_topology(initLink("topology", "link to topology")) {
        l_topology.setPath("@.");
    }

    void init() {
        auto f = [=](const EdgeElement * elmt, double f0,double f1) -> BaseProximity::SPtr {
            return BaseProximity::SPtr(new DefaultEdgeProximity<DataTypes>(this->getState(),
                                                                        elmt->getP0(),elmt->getP1(),
                                                                        f0,f1));
        };

        for (unsigned i=0;i<this->l_topology->getNbEdges();i++) {
            auto edge = this->l_topology->getEdge(i);

            EdgeElement::SPtr elmt = this->createElement(i,edge[0],edge[1],f);
            m_elements.push_back(elmt);
        }
    }

    inline BaseElement::Iterator begin(Index eid = 0) const override {
        return BaseElement::Iterator(new TDefaultElementIterator(m_elements,eid));
    }

//    inline void draw(const core::visual::VisualParams * vparams) {
//        this->drawNormals(vparams);

////        if (! vparams->displayFlags().getShowCollisionModels()) return;
//        if (! vparams->displayFlags().getShowCollisionModels()) return ;
//        const sofa::type::RGBAColor & color = this->d_color.getValue();
//        if (color[3] == 0.0) return;

//        glDisable(GL_LIGHTING);

//        glBegin(GL_LINES);
//        glColor4fv(color.data());
//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());

//        for (auto it=this->begin();it!=this->end();it++) {
//            const sofa::topology::Edge & edge = this->l_topology->getEdge(it->id());

//            glColor4f(1.0,0.0,0.0,1.0);
//            glVertex3dv(pos[edge[0]].data());
//            glColor4f(0.0,0.0,1.0,1.0);
//            glVertex3dv(pos[edge[1]].data());
//        }
//        glEnd();
//    }

private:
    std::vector<EdgeElement::SPtr> m_elements;
};

}

}

#pragma once

#include <sofa/collisionAlgorithm/algorithm/BaseClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

class ManualBindAlgorithm : public BaseAlgorithm
{
public:
    SOFA_CLASS(ManualBindAlgorithm, BaseAlgorithm);

    core::objectmodel::SingleLink<ManualBindAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<ManualBindAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;

    Data<sofa::type::vector<int> > d_bindFrom;
    Data<sofa::type::vector<int> > d_bindDest;

    Data<bool> d_drawCollision ;
    Data<DetectionOutput> d_output;

    ManualBindAlgorithm()
    : l_from(initLink("from", "link to from geometry"))
    , l_dest(initLink("dest", "link to dest geometry"))
    , d_bindFrom (initData(&d_bindFrom, "bindFirst", "draw collision"))
    , d_bindDest (initData(&d_bindDest, "bindDest", "draw collision"))
    , d_drawCollision (initData(&d_drawCollision, false, "drawcollision", "draw collision"))
    , d_output(initData(&d_output,"output", "output of the collision detection")) {}

    void draw(const core::visual::VisualParams* /*vparams*/) {
        if (! d_drawCollision.getValue()) return;
        glDisable(GL_LIGHTING);
        glColor4f(0,1,0,1);

        glBegin(GL_LINES);
        DetectionOutput output = d_output.getValue() ;
        for (unsigned i=0;i<output.size();i++) {
            glVertex3dv(output[i].first->getPosition().data());
            glVertex3dv(output[i].second->getPosition().data());
        }
        glEnd();
    }

    void doDetection() {
        if (l_from == NULL) return;
        if (l_dest == NULL) return;

        DetectionOutput & output = *d_output.beginEdit();
        output.clear();

        unsigned sz = std::min(d_bindFrom.getValue().size(),d_bindDest.getValue().size());

        for (unsigned i=0;i<sz;i++) {
            auto itfrom = l_from->begin(d_bindFrom.getValue()[i]);
            auto itdest = l_dest->begin(d_bindDest.getValue()[i]);

            if (itfrom == l_from->end()) continue;
            if (itdest == l_dest->end()) continue;

            BaseProximity::SPtr first = itfrom->createProximity();
            BaseProximity::SPtr second = itdest->createProximity();

            output.add(first,second);
        }

        d_output.endEdit();
    }

};


}

}

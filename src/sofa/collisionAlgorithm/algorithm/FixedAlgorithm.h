#pragma once

#include <sofa/collisionAlgorithm/algorithm/BaseClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

#include <sofa/collisionAlgorithm/data/DataDistanceMeasure.h>

namespace sofa
{

namespace collisionAlgorithm
{

class FixedAlgorithm : public BaseClosestProximityAlgorithm
{
public:
    SOFA_CLASS(FixedAlgorithm, BaseClosestProximityAlgorithm);

    core::objectmodel::SingleLink<FixedAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;

    Data<helper::vector<int> > d_bindFrom;
    Data<helper::vector<int> > d_bindDest;

    Data<bool> d_drawCollision ;
    Data<DetectionOutput> d_output;

    FixedAlgorithm()
    : l_from(initLink("from", "link to from geometry"))
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

        DetectionOutput & output = *d_output.beginEdit();
        output.clear();

        for(auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++)
        {

            BaseProximity::SPtr first = itfrom->center();
            BaseProximity::SPtr second = std::shared_ptr<BaseProximity>(new FixedProximity(first->getPosition()));

            output.add(first,second);
        }

        d_output.endEdit();
    }

};


}

}

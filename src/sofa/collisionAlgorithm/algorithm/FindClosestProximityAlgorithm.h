#pragma once

#include <sofa/collisionAlgorithm/algorithm/BaseClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

#include <sofa/collisionAlgorithm/data/DataDistanceMeasure.h>

namespace sofa
{

namespace collisionAlgorithm
{

class FindClosestProximityAlgorithm : public BaseClosestProximityAlgorithm
{
public:
    SOFA_CLASS(FindClosestProximityAlgorithm, BaseClosestProximityAlgorithm);

    core::objectmodel::SingleLink<FindClosestProximityAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<FindClosestProximityAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;
    Data<bool> d_drawCollision ;
    Data<DetectionOutput> d_output;

    FindClosestProximityAlgorithm()
    : l_from(initLink("from", "link to from geometry"))
    , l_dest(initLink("dest", "link to dest geometry"))
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
    //    size_t i = 0, fail = 0 ; //debug purposes
        for (auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++) {
            PairDetection min_pair = findClosestPoint(*itfrom,l_dest.get());
    //        i++ ;
            if (min_pair.first == nullptr || min_pair.second == nullptr) {
    //            fail++ ;
                continue;
            }

            output.add(min_pair.first,min_pair.second);
        }
    //    std::cout << i << ':' << fail << this->getName() << std::endl ;
        d_output.endEdit();
    }

};


}

}

#pragma once

#include <sofa/collisionAlgorithm/algorithm/BaseClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

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
    Data<helper::vector<unsigned>> d_matchIdFrom;
    Data<helper::vector<unsigned>> d_matchIdDest;

    FindClosestProximityAlgorithm()
    : l_from(initLink("from", "link to from geometry"))
    , l_dest(initLink("dest", "link to dest geometry"))
    , d_drawCollision (initData(&d_drawCollision, true, "drawcollision", "draw collision"))
    , d_output(initData(&d_output,"output", "output of the collision detection"))
    , d_matchIdFrom(initData(&d_matchIdFrom,"matchIdFrom", "from marker ID"))
    , d_matchIdDest(initData(&d_matchIdDest,"matchIdDest", "dest marker ID")) {}

    void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowCollisionModels() && ! d_drawCollision.getValue()) return;
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

        helper::vector<unsigned> & matchIdFrom = *d_matchIdFrom.beginEdit();
        helper::vector<unsigned> & matchIdDest = *d_matchIdDest.beginEdit();

        for (auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++) {
            PairDetection min_pair = findClosestPoint(*itfrom,l_dest.get());
            if (min_pair.first == nullptr || min_pair.second == nullptr) {
                continue;
            }

            output.add(min_pair.first,min_pair.second);

            matchIdFrom.push_back(min_pair.first->getElementId());
            matchIdDest.push_back(min_pair.second->getElementId());

        }
        d_output.endEdit();
        d_matchIdFrom.endEdit();
        d_matchIdDest.endEdit();
    }

};


}

}

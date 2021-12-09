#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

class FindClosestProximityAlgorithm : public BaseAlgorithm
{
public:
    SOFA_CLASS(FindClosestProximityAlgorithm, BaseAlgorithm);

    core::objectmodel::SingleLink<FindClosestProximityAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<FindClosestProximityAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;
    Data<bool> d_drawCollision ;
    Data<DetectionOutput> d_output;
    Data<sofa::type::vector<double> > d_outputDist;

    FindClosestProximityAlgorithm()
    : l_from(initLink("from", "link to from geometry"))
    , l_dest(initLink("dest", "link to dest geometry"))
    , d_drawCollision (initData(&d_drawCollision, true, "drawcollision", "draw collision"))
    , d_output(initData(&d_output,"output", "output of the collision detection"))
    , d_outputDist(initData(&d_outputDist,"outputDist", "Distance of the outpu pair of detections"))
    , m_distance([=](BaseProximity::SPtr a,BaseProximity::SPtr b){ return (a->getPosition()-b->getPosition()).norm(); })
    {}

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
        for (auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++) {
            auto pfrom = itfrom->createProximity();
            if (pfrom == nullptr) continue;

            double min_dist = std::numeric_limits<double>::max();
            PairDetection min_pair;

            for (auto itdest=l_dest->begin();itdest!=l_dest->end();itdest++) {
                auto pdest = itdest->createProximity();
                if (pdest == nullptr) continue;

                double d = m_distance(pfrom,pdest);
                if (d < min_dist) min_pair = PairDetection(pfrom,pdest);
            }

            if (min_pair.first == nullptr || min_pair.second == nullptr) continue;

            output.add(min_pair.first,min_pair.second);
        }
        d_output.endEdit();
    }

private:
    std::function<double(BaseProximity::SPtr,BaseProximity::SPtr)> m_distance;

};


}

}
